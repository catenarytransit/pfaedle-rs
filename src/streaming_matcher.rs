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
    pub fn new(
        osm_path: &Path,
        cache_size: usize,
        light_osm: LightOsmData,
        skip_small_roads: bool,
    ) -> Result<Self> {
        // 1. Identify Resources (Global scan)
        use crate::osm_load::OsmBuilder;
        let (pre_rels, ways_in_rels, ferrys, needed_nodes) =
            OsmBuilder::identify_resources(osm_path, skip_small_roads)?;

        // 2. Split PBF into tiles
        use crate::osm_split::OsmSplitter;
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();
        osm_path.hash(&mut hasher);
        let hash = hasher.finish();

        // Cache directory for split tiles
        let cache_dir = std::path::PathBuf::from(format!("/tmp/pfaedle-split-{:x}", hash));
        if cache_dir.exists() {
            // Optional: check if valid? Or just wipe?
            // For safety in dev, let's wipe to ensure fresh logic
            std::fs::remove_dir_all(&cache_dir).ok();
        }

        let splitter = OsmSplitter::new(osm_path, &cache_dir)?;
        splitter.split_pbf(&needed_nodes, &ways_in_rels, &ferrys)?;

        // 3. Init cache
        let tile_cache = TileCache::new_with_split_dir(&cache_dir, cache_size)?;

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

        // Sort patterns by agency, with agencies ordered by TSP on their centroids
        // This maximizes cache locality: routes from the same agency share tiles
        println!("  Grouping by agency and computing TSP order...");
        
        // 1. Group patterns by agency
        let mut agency_patterns: AHashMap<String, Vec<(&StopPattern, &Vec<String>)>> = AHashMap::new();
        for (pattern, trips) in patterns {
            let agency_name = get_pattern_agency_name(gtfs, pattern);
            agency_patterns.entry(agency_name).or_default().push((pattern, trips));
        }
        
        // 2. Compute centroid for each agency
        let mut agency_centroids: Vec<(String, (f64, f64))> = agency_patterns
            .keys()
            .map(|agency| {
                let patterns = &agency_patterns[agency];
                let mut all_coords: Vec<(f64, f64)> = Vec::new();
                for (pattern, _) in patterns {
                    for stop_id in &pattern.stop_ids {
                        if let Some(stop) = gtfs.gtfs.stops.get(stop_id) {
                            if let (Some(lon), Some(lat)) = (stop.longitude, stop.latitude) {
                                all_coords.push((lon, lat));
                            }
                        }
                    }
                }
                let centroid = crate::hilbert::compute_centroid(&all_coords);
                (agency.clone(), centroid)
            })
            .collect();
        
        // 3. Solve TSP to order agencies (if more than 2 agencies)
        let ordered_agencies: Vec<String> = if agency_centroids.len() <= 2 {
            // Trivial case: just sort alphabetically
            agency_centroids.sort_by(|a, b| a.0.cmp(&b.0));
            agency_centroids.into_iter().map(|(name, _)| name).collect()
        } else {
            // Use TSP to find optimal ordering
            let cities: Vec<(f64, f64)> = agency_centroids.iter().map(|(_, c)| *c).collect();
            let tour = travelling_salesman::simulated_annealing::solve(
                &cities,
                time::Duration::milliseconds(200),
            );
            println!("    TSP tour distance: {:.2}, route: {:?}", tour.distance, tour.route);
            tour.route.into_iter().map(|i| agency_centroids[i].0.clone()).collect()
        };
        
        println!("    Agency order ({} agencies): {:?}", ordered_agencies.len(), 
            ordered_agencies.iter().take(5).collect::<Vec<_>>());
        
        // 4. Flatten patterns in TSP order
        let mut patterns_vec: Vec<(&StopPattern, &Vec<String>)> = Vec::with_capacity(total);
        for agency in &ordered_agencies {
            if let Some(mut agency_pats) = agency_patterns.remove(agency) {
                patterns_vec.append(&mut agency_pats);
            }
        }

        // 2. Sequential Processing
        // We cannot use parallel iterator easily here because TileCache is mutable (LRU updates).
        // However, we can use a Mutex, but contention might be high.
        // Given that we are waiting for disk I/O for tiles (initially), sequential might be safer for memory stability.
        // Or we could use a thread pool with a shared cache wrapped in Mutex.
        // Let's try sequential first to guarantee memory safety.

        // 2. Parallel Batch Processing
        // We accumulate patterns into batches to share tile loading.
        // Routes from the same agency (sequential in list) are likely close, sharing tiles.
        
        const BATCH_SIZE: usize = 32;       // Reduced from 64 for memory efficiency
        const MAX_BATCH_TILES: usize = 80;  // Reduced from 200 to prevent OOM
        
        let mut results = AHashMap::new();
        let total_patterns = patterns_vec.len();
        
        let light_osm = &self.light_osm;
        
        // Helper to process a batch
        let process_batch = |batch: &[(&StopPattern, &Vec<String>)], 
                             tile_cache: &mut TileCache, 
                             results: &mut AHashMap<StopPattern, ShapeResult>| {
            
            // 1. Identify all tiles needed for this batch
            let mut batch_tiles = ahash::AHashSet::new();
            let mut batch_info = Vec::with_capacity(batch.len());

            for (pattern, _trips) in batch {
                 // Get coords
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
                    
                 if stop_coords.len() >= 2 {
                     let tiles = crate::tile::compute_route_tiles(&stop_coords);
                     for t in &tiles {
                         batch_tiles.insert(*t);
                     }
                     batch_info.push((pattern, stop_coords));
                 }
            }
            
            let unique_tiles: Vec<_> = batch_tiles.into_iter().collect();
            
            if unique_tiles.is_empty() {
                return;
            }

            // 2. Load merged graph for the whole batch
            // This is the heavy I/O step, done once per batch
            if let Ok(merged_arc) = tile_cache.merge_tiles_cached(&unique_tiles) {
                
                // 3. Parallel Match
                let batch_results: Vec<_> = batch_info
                    .par_iter()
                    .map(|(pattern, stop_coords)| {
                         // Build context locally per thread
                         let mut ctx = PathfinderContext::new();
                         
                         // Extract route info for matching preferences
                         let sample_trip_id = gtfs.patterns.get(pattern).and_then(|t| t.first());
                         let trip = sample_trip_id.and_then(|tid| gtfs.gtfs.trips.get(tid));
                         let route = trip.and_then(|t| gtfs.gtfs.routes.get(&t.route_id));
                         
                         let agency_name = route.and_then(|r| r.agency_id.as_ref()).and_then(|id| { // Nested and_then fixes the double-pass? No, standard logic
                             gtfs.gtfs.agencies.iter().find(|a| a.id.as_ref() == Some(id)).map(|a| a.name.to_lowercase())
                         });
                         
                         let preferred_match = TransitMatch {
                             short_name: route.and_then(|r| r.short_name.as_ref()).map(|s| s.to_lowercase()),
                             long_name: route.and_then(|r| r.long_name.as_ref()).map(|s| s.to_lowercase()),
                             operator: agency_name,
                         };

                         // Match!
                         if let Some(geometry) = SegmentMatcher::match_route_with_graph(
                             &merged_arc,
                             stop_coords,
                             Some(&preferred_match),
                             &mut ctx
                         ) {
                             // Success
                             let mut hasher = DefaultHasher::new();
                             pattern.hash(&mut hasher);
                             let shape_id = format!("shape_{}", hasher.finish());
                             
                             Some((
                                 (*pattern).clone(),
                                 ShapeResult {
                                     shape_id,
                                     points: geometry,
                                     matched_route_color: light_osm.find_color(
                                         preferred_match.short_name.as_deref(),
                                         preferred_match.long_name.as_deref(),
                                         preferred_match.operator.as_deref()
                                     ),
                                 }
                             ))
                         } else {
                             None
                         }
                    })
                    .collect();
                
                // 4. Collect results
                for res in batch_results {
                    if let Some((pat, shape_res)) = res {
                        results.insert(pat.clone(), shape_res);
                    }
                }
                
                // Explicitly drop the merged graph to free memory immediately
                drop(merged_arc);
            } else {
                // Log failure with details to help diagnose
                eprintln!(
                    "    WARNING: Failed to load {} tiles for batch of {} patterns. Skipping batch.",
                    unique_tiles.len(),
                    batch.len()
                );
            }
        };

        // Main Loop
        let mut current_batch = Vec::new();
        let mut current_batch_tiles = ahash::AHashSet::new(); 
        
        for (i, item) in patterns_vec.iter().enumerate() {
            if i % 100 == 0 {
                println!("  Processed {}/{} patterns... (Results: {})", i, total_patterns, results.len());
            }
            
            // Estimate tiles for this pattern (quick approximation or full compute?)
            // We need full compute to check capacity.
            let (pattern, _) = item;
            let stop_coords: Vec<Point<f64>> = pattern.stop_ids.iter()
                .filter_map(|sid| gtfs.gtfs.stops.get(sid))
                .filter_map(|s| if let (Some(x), Some(y)) = (s.longitude, s.latitude) { Some(Point::new(x, y)) } else { None })
                .collect();
            
            // Just compute tiles for checking batch limits
            let tiles = if stop_coords.len() >= 2 {
                crate::tile::compute_route_tiles(&stop_coords)
            } else {
                Vec::new()
            };
            
            let mut new_tiles_count = 0;
            for t in &tiles {
                if !current_batch_tiles.contains(t) {
                    new_tiles_count += 1;
                }
            }
            
            // Check if adding this pattern breaks batch limits
            if !current_batch.is_empty() && (
               current_batch.len() >= BATCH_SIZE || 
               current_batch_tiles.len() + new_tiles_count > MAX_BATCH_TILES
            ) {
                 // FLUSH BATCH
                 process_batch(&current_batch, &mut self.tile_cache, &mut results);
                 current_batch.clear();
                 current_batch_tiles.clear();
            }
            
            // Add to current
            current_batch.push(*item);
            for t in tiles {
                current_batch_tiles.insert(t);
            }
        }
        
        // Flush last batch
        if !current_batch.is_empty() {
             process_batch(&current_batch, &mut self.tile_cache, &mut results);
        }

        println!("Streaming match complete. Found {} shapes.", results.len());
        let non_empty = results.values().filter(|r| !r.points.is_empty()).count();
        println!("  {} shapes have non-empty geometry.", non_empty);
        if non_empty < results.len() {
            println!("  WARNING: {} shapes have EMPTY geometry!", results.len() - non_empty);
        }
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

        // 3. Match using SegmentMatcher with preloaded tiles (MUCH faster)
        let mut segment_matcher = SegmentMatcher::new(&mut self.tile_cache);

        // Attempt match using preloaded route - loads all tiles once upfront
        let geometry_opt = segment_matcher.match_route_preloaded(&stop_coords, Some(&preferred_match));

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

/// Helper function to get the agency name for a pattern
fn get_pattern_agency_name(gtfs: &GtfsData, pattern: &StopPattern) -> String {
    gtfs.patterns.get(pattern)
        .and_then(|trips| trips.first())
        .and_then(|trip_id| gtfs.gtfs.trips.get(trip_id))
        .and_then(|trip| gtfs.gtfs.routes.get(&trip.route_id))
        .and_then(|route| route.agency_id.as_ref())
        .and_then(|agency_id| {
            gtfs.gtfs.agencies.iter()
                .find(|a| a.id.as_ref() == Some(agency_id))
                .map(|a| a.name.to_lowercase())
        })
        .unwrap_or_else(|| String::from("zzz_unknown"))
}
