use ahash::AHashMap;
use anyhow::Result;
use geo::Point;
use rayon::prelude::*;
use std::path::Path;
use std::sync::{Arc, Mutex};

use crate::graph::MODE_BUS;
use crate::gtfs_load::{GtfsData, StopPattern};
use crate::matcher::{BinaryShapeRecord, ShapeResult, build_match_edge_index, match_one_pattern};
use crate::osm_load::{LightOsmData, OsmBuilder, OsmData};
use crate::pathfinding::PathfinderContext;
use crate::tile_loader::{RoadProfile, TileCache, TileCoord, compute_route_tiles};

/// Bus/coach matcher that keeps the disk tiling pre-pass but uses the same
/// collapsed edge-projection + RouterImpl path as the full upstream-style graph.
pub struct StreamingMatcher {
    tile_cache: TileCache,
    light_osm: Arc<LightOsmData>,
    cache_dir: std::path::PathBuf,
}

impl StreamingMatcher {
    pub fn new(
        osm_path: &Path,
        _cache_size: usize,
        light_osm: LightOsmData,
        skip_small_roads: bool,
    ) -> Result<Self> {
        // Upstream constructs a mode-specific graph. Restrict the split resource
        // pass to bus infrastructure/relations before writing any tile buckets.
        let mut resources =
            OsmBuilder::identify_resources_for_modes(osm_path, skip_small_roads, MODE_BUS)?;
        // The splitter only needs the filtered flat IDs. Bus line metadata is
        // supplied by LightOsmData, so release the heavyweight relation Tags
        // and member vectors before the two disk-split passes.
        resources.pre_relations.clear();
        resources.pre_relations.shrink_to_fit();
        resources.ways_in_relations.clear();
        resources.ways_in_relations.shrink_to_fit();
        resources.ways_in_ferry_relations.clear();
        resources.ways_in_ferry_relations.shrink_to_fit();

        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        let mut hasher = DefaultHasher::new();
        osm_path.hash(&mut hasher);
        let hash = hasher.finish();
        let cache_dir = std::path::PathBuf::from(format!("/tmp/pfaedle-split-{:x}", hash));
        if cache_dir.exists() {
            std::fs::remove_dir_all(&cache_dir).ok();
        }

        let splitter = crate::osm_split::OsmSplitter::new(osm_path, &cache_dir)?;
        splitter.split_pbf(&mut resources)?;

        let light_osm = Arc::new(light_osm);
        let tile_cache = TileCache::new_with_split_dir(&cache_dir, light_osm.clone())?;
        Ok(Self {
            tile_cache,
            light_osm,
            cache_dir,
        })
    }

    pub fn match_all(
        &mut self,
        gtfs: &GtfsData,
        patterns: Vec<(&StopPattern, &Vec<String>)>,
        cache_file_path: &std::path::Path,
    ) -> AHashMap<StopPattern, ShapeResult> {
        let total = patterns.len();
        println!(
            "Streaming/upstream matching for {} bus-like patterns...",
            total
        );

        // C++ builds separate graphs for the common bus profile and the
        // `[coach]` override profile. Keep that separation so routing levels and
        // collapse boundaries match upstream, while sharing the same disk split.
        let mut agency_patterns: AHashMap<
            (String, RoadProfile),
            Vec<(&StopPattern, &Vec<String>)>,
        > = AHashMap::new();
        for (pattern, trips) in patterns {
            agency_patterns
                .entry((
                    get_pattern_agency_name(gtfs, pattern),
                    road_profile_for_pattern(pattern),
                ))
                .or_default()
                .push((pattern, trips));
        }

        let mut agencies: Vec<_> = agency_patterns.keys().cloned().collect();
        agencies.sort();

        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(cache_file_path)
            .expect("Failed to open shape cache file");
        let writer = Mutex::new(std::io::BufWriter::new(file));
        let mut results = AHashMap::new();

        for (agency_idx, (agency_name, road_profile)) in agencies.iter().enumerate() {
            let Some(patterns) = agency_patterns.get(&(agency_name.clone(), *road_profile)) else {
                continue;
            };
            println!(
                "  Processing {:?} agency {}/{} ({})",
                road_profile,
                agency_idx + 1,
                agencies.len(),
                agency_name
            );

            // Keep geometrically nearby patterns together, but cap each merged
            // graph at 50 tiles. Unlike the old implementation these groups are
            // not cached after use.
            let mut pattern_info: Vec<((&StopPattern, &Vec<String>), Vec<TileCoord>, (f64, f64))> =
                Vec::new();
            for &(pattern, trips) in patterns {
                let stop_coords = pattern_stop_coords(gtfs, pattern);
                if stop_coords.len() < 2 {
                    continue;
                }
                let route_tiles = compute_route_tiles(&stop_coords);
                let sum_lon: f64 = stop_coords.iter().map(|point| point.x()).sum();
                let sum_lat: f64 = stop_coords.iter().map(|point| point.y()).sum();
                let centroid = (
                    sum_lon / stop_coords.len() as f64,
                    sum_lat / stop_coords.len() as f64,
                );
                pattern_info.push(((pattern, trips), route_tiles, centroid));
            }
            pattern_info
                .sort_by_cached_key(|(_, _, (lon, lat))| crate::hilbert::hilbert_index(*lon, *lat));

            let mut groups: Vec<(
                Vec<(&StopPattern, &Vec<String>)>,
                ahash::AHashSet<TileCoord>,
            )> = Vec::new();
            let mut current_patterns = Vec::new();
            let mut current_tiles = ahash::AHashSet::new();

            for (pattern_data, route_tiles, _) in pattern_info {
                let additional = route_tiles
                    .iter()
                    .filter(|tile| !current_tiles.contains(*tile))
                    .count();
                if !current_patterns.is_empty() && current_tiles.len() + additional > 50 {
                    groups.push((current_patterns, current_tiles));
                    current_patterns = Vec::new();
                    current_tiles = ahash::AHashSet::new();
                }
                current_patterns.push(pattern_data);
                current_tiles.extend(route_tiles);
            }
            if !current_patterns.is_empty() {
                groups.push((current_patterns, current_tiles));
            }

            let group_count = groups.len();
            for (group_idx, (group_patterns, group_tiles)) in groups.into_iter().enumerate() {
                let mut tiles: Vec<_> = group_tiles.into_iter().collect();
                tiles.sort_by_key(|tile| (tile.x, tile.y));
                println!(
                    "    Group {}/{}: {} patterns, {} tiles",
                    group_idx + 1,
                    group_count,
                    group_patterns.len(),
                    tiles.len()
                );

                let graph = match self.tile_cache.merge_tiles(&tiles, *road_profile) {
                    Ok(graph) => graph,
                    Err(error) => {
                        eprintln!("    WARNING: failed to build bus group graph: {error:#}");
                        continue;
                    }
                };

                // match_one_pattern only needs `osm.graph`. Wrapping the owned
                // graph avoids a second bus-specific routing implementation.
                let osm = OsmData {
                    graph,
                    timestamp: "split".to_string(),
                    spatial_tree: None,
                    osm_filepath: std::path::PathBuf::new(),
                    relations: Vec::new(),
                    node_to_relations: AHashMap::new(),
                };
                let edge_index = build_match_edge_index(&osm, MODE_BUS);
                println!(
                    "      Collapsed bus graph: {} nodes, {} directed edges, edge index {}",
                    osm.graph.nodes.len(),
                    osm.graph.edges.len(),
                    edge_index.size()
                );

                // Geometry is serialized inside the worker and discarded
                // immediately. The old code collected every route geometry in a
                // Vec before writing, which could retain hundreds of MB per group.
                let small_results: Vec<_> = group_patterns
                    .par_iter()
                    .map_init(PathfinderContext::new, |ctx, (pattern, trips)| {
                        let mut matched =
                            match_one_pattern(gtfs, &osm, &edge_index, pattern, trips, ctx);
                        ctx.discard_if_oversized(500_000);

                        let Some((matched_pattern, mut shape_result, points)) = matched.take()
                        else {
                            return None;
                        };

                        shape_result.matched_route_color =
                            route_color(gtfs, pattern, agency_name, &self.light_osm);

                        if !shape_result.empty_geometry {
                            let mut writer = writer.lock().ok()?;
                            for (sequence, (lat, lon)) in points.into_iter().enumerate() {
                                let record = BinaryShapeRecord {
                                    shape_id: shape_result.shape_id.clone(),
                                    shape_pt_lat: lat,
                                    shape_pt_lon: lon,
                                    shape_pt_sequence: sequence + 1,
                                };
                                if bincode::serialize_into(&mut *writer, &record).is_err() {
                                    return None;
                                }
                            }
                        }

                        Some((matched_pattern, shape_result))
                    })
                    .collect();

                for result in small_results.into_iter().flatten() {
                    results.insert(result.0, result.1);
                }
                // `osm` and its sole edge R-tree are dropped here. No tile graph
                // or merged graph cache can retain a duplicate copy.
            }
        }

        use std::io::Write;
        if let Ok(mut writer) = writer.lock() {
            writer.flush().expect("Failed to flush shape cache file");
        }

        println!("Streaming match complete. Found {} shapes.", results.len());
        results
    }
}

impl Drop for StreamingMatcher {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.cache_dir);
    }
}

fn pattern_stop_coords(gtfs: &GtfsData, pattern: &StopPattern) -> Vec<Point<f64>> {
    pattern
        .stop_ids
        .iter()
        .filter_map(|stop_id| gtfs.gtfs.stops.get(stop_id))
        .filter_map(|stop| match (stop.longitude, stop.latitude) {
            (Some(lon), Some(lat)) => Some(Point::new(lon, lat)),
            _ => None,
        })
        .collect()
}

fn route_color(
    gtfs: &GtfsData,
    pattern: &StopPattern,
    agency_name: &str,
    light_osm: &LightOsmData,
) -> Option<String> {
    let trip_id = gtfs.patterns.get(pattern)?.first()?;
    let trip = gtfs.gtfs.trips.get(trip_id)?;
    let route = gtfs.gtfs.routes.get(&trip.route_id)?;
    let short = route.short_name.as_ref().map(|value| value.to_lowercase());
    let long = route.long_name.as_ref().map(|value| value.to_lowercase());
    light_osm.find_color(short.as_deref(), long.as_deref(), Some(agency_name))
}

fn road_profile_for_pattern(pattern: &StopPattern) -> RoadProfile {
    use crate::mots::{MotCategory, map_route_type_to_category};
    if pattern.route_type.map(map_route_type_to_category) == Some(MotCategory::Coach) {
        RoadProfile::Coach
    } else {
        RoadProfile::Bus
    }
}

fn get_pattern_agency_name(gtfs: &GtfsData, pattern: &StopPattern) -> String {
    gtfs.patterns
        .get(pattern)
        .and_then(|trips| trips.first())
        .and_then(|trip_id| gtfs.gtfs.trips.get(trip_id))
        .and_then(|trip| gtfs.gtfs.routes.get(&trip.route_id))
        .and_then(|route| route.agency_id.as_ref())
        .and_then(|agency_id| {
            gtfs.gtfs
                .agencies
                .iter()
                .find(|agency| agency.id.as_ref() == Some(agency_id))
                .map(|agency| agency.name.to_lowercase())
        })
        .unwrap_or_else(|| "zzz_unknown".to_string())
}
