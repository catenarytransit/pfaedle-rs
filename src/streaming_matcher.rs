use ahash::AHashMap;
use anyhow::{Context, Result};
use geo::Point;
use gtfs_structures::RouteType;
use rayon::prelude::*;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use crate::graph::{MODE_BUS, MODE_FERRY, MODE_GONDOLA, MODE_RAIL, MODE_SUBWAY, MODE_TRAM};
use crate::gtfs_load::{GtfsData, StopPattern};
use crate::hilbert::route_hilbert_index;
use crate::matcher::ShapeResult;
use crate::osm_load::LightOsmData;
use crate::pathfinding::{PathfinderContext, TransitMatch};
use crate::segment_matcher::SegmentMatcher;
use crate::tile::TileCache;

/// Streaming matcher for memory-efficient bus processing.
pub struct StreamingMatcher {
    tile_cache: TileCache,
    light_osm: Arc<LightOsmData>,
}

impl StreamingMatcher {
    /// Create a new streaming matcher.
    /// `osm_path` is used to load tiles on demand.
    /// `cache_size` is the max number of tiles to keep in memory (e.g., 100).
    pub fn new(osm_path: &Path, cache_size: usize, light_osm: LightOsmData) -> Result<Self> {
        let tile_cache = TileCache::new_with_disk_cache(osm_path, cache_size)?;
        Ok(Self {
            tile_cache,
            light_osm: Arc::new(light_osm),
        })
    }

    /// Match all patterns using a streaming approach.
    /// Returns a map of pattern -> shape result.
    pub fn match_all(
        &mut self,
        gtfs: &GtfsData,
        patterns: Vec<(&StopPattern, &Vec<String>)>,
    ) -> AHashMap<StopPattern, ShapeResult> {
        let total = patterns.len();
        println!("Streaming matching for {} patterns...", total);

        // Sort patterns by Hilbert curve to maximize cache locality
        let mut patterns_vec: Vec<_> = patterns.into_iter().collect();

        // 1. Sort
        println!("  Sorting by Hilbert index...");
        patterns_vec.sort_by_cached_key(|(pattern, _)| {
            let coords: Vec<_> = pattern
                .stop_ids
                .iter()
                .filter_map(|sid| gtfs.gtfs.stops.get(sid))
                .filter_map(|s| {
                    if let (Some(lon), Some(lat)) = (s.longitude, s.latitude) {
                        Some((lon, lat))
                    } else {
                        None
                    }
                })
                .collect();
            route_hilbert_index(&coords)
        });

        // 2. Sequential Processing
        // We cannot use parallel iterator easily here because TileCache is mutable (LRU updates).
        // However, we can use a Mutex, but contention might be high.
        // Given that we are waiting for disk I/O for tiles (initially), sequential might be safer for memory stability.
        // Or we could use a thread pool with a shared cache wrapped in Mutex.
        // Let's try sequential first to guarantee memory safety.

        // Progress tracking
        let processed_count = 0;
        let mut results = AHashMap::new();

        for (i, (pattern, trips)) in patterns_vec.iter().enumerate() {
            if i % 50 == 0 {
                println!(
                    "  Processed {}/{} ({:.1}%) - Cache Usage: {} entries",
                    i,
                    total,
                    (i as f64 / total as f64) * 100.0,
                    self.tile_cache.len()
                );
            }

            if let Some(result) = self.match_pattern(gtfs, pattern) {
                results.insert((*pattern).clone(), result);
            }
        }

        println!("Streaming match complete. Found {} shapes.", results.len());
        results
    }

    /// Match a single pattern by loading necessary tiles.
    fn match_pattern(&mut self, gtfs: &GtfsData, pattern: &StopPattern) -> Option<ShapeResult> {
        // 1. Extract Info
        let sample_trip_id = gtfs.patterns.get(pattern)?.first()?;
        let trip = gtfs.gtfs.trips.get(sample_trip_id)?;
        let route = gtfs.gtfs.routes.get(&trip.route_id)?;

        let agency_name = route.agency_id.as_ref().and_then(|id| {
            gtfs.gtfs
                .agencies
                .iter()
                .find(|a| a.id.as_ref() == Some(id))
                .map(|a| a.name.to_lowercase())
        });

        let preferred_match = TransitMatch {
            short_name: route.short_name.as_ref().map(|s| s.to_lowercase()),
            long_name: route.long_name.as_ref().map(|s| s.to_lowercase()),
            operator: agency_name,
        };

        // 2. Get Coords
        let stop_coords: Vec<Point<f64>> = pattern
            .stop_ids
            .iter()
            .filter_map(|sid| gtfs.gtfs.stops.get(sid))
            .filter_map(|s| {
                if let (Some(lon), Some(lat)) = (s.longitude, s.latitude) {
                    Some(Point::new(lon, lat))
                } else {
                    None
                }
            })
            .collect();

        if stop_coords.len() < 2 {
            return None;
        }

        // 3. Match using SegmentMatcher (which uses TileCache)
        let mut segment_matcher = SegmentMatcher::new(&mut self.tile_cache);

        // Attempt match
        let geometry_opt = segment_matcher.match_route(&stop_coords, Some(&preferred_match));

        // If failed, return None (or we could return straight line? pfaedle usually wants matched shape)
        // But original pfaedle falls back to "globally optimal" if relation match fails.
        // Our SegmentMatcher does pathfinding. If it returns None, it means pathfinding failed.
        // We should probably just return None here.

        let geometry = geometry_opt?;

        // 4. Color Matching
        // Since we didn't load relations into the graph tiles (only ways), we use LightOsmData for colors.
        // This is an approximation: we check if any relation matching this route exists in the light data.
        let matched_colour = self.light_osm.find_color(
            preferred_match.short_name.as_deref(),
            preferred_match.long_name.as_deref(),
            preferred_match.operator.as_deref(),
        );

        // 5. Generate ID
        let mut hasher = DefaultHasher::new();
        pattern.hash(&mut hasher);
        let shape_id = format!("shape_{}", hasher.finish());

        Some(ShapeResult {
            shape_id,
            points: geometry,
            matched_route_color: matched_colour,
        })
    }
}
