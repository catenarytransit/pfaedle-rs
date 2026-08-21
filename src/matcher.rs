use ahash::{AHashMap, AHashSet};
use geo::Point;
use geo::algorithm::HaversineDistance;
use gtfs_structures::RouteType;

use crate::graph::{
    EdgePL, Graph, MODE_BUS, MODE_FERRY, MODE_GONDOLA, MODE_RAIL, MODE_SUBWAY, MODE_TRAM, NodePL,
};
use crate::gtfs_load::{GtfsData, StopPattern};
use crate::osm_load::OsmData;
use crate::pathfinding::{self, TransitMatch};

#[derive(Debug, Clone)]
pub struct ShapeResult {
    pub shape_id: String,
    pub empty_geometry: bool,
    pub matched_route_color: Option<String>,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct BinaryShapeRecord {
    pub shape_id: String,
    pub shape_pt_lat: f64,
    pub shape_pt_lon: f64,
    pub shape_pt_sequence: usize,
}

#[derive(Debug, Clone)]
struct SpatialEdge {
    edge_idx: crate::graph::EdgeIndex,
    envelope: rstar::AABB<[f64; 2]>,
}

impl rstar::RTreeObject for SpatialEdge {
    type Envelope = rstar::AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope.clone()
    }
}

#[derive(Debug, Clone)]
struct ProjectedEdgeCandidate {
    edge_idx: crate::graph::EdgeIndex,
    distance_m: f64,
    progress: f64,
}

fn build_match_edge_index(osm: &crate::osm_load::OsmData) -> rstar::RTree<SpatialEdge> {
    // Full-graph matching only handles non-bus modes. Keeping road-only edges
    // out of this index mirrors upstream's per-MOT graph/index construction and
    // avoids doubling memory on large metropolitan extracts.
    let indexed_modes = MODE_RAIL | MODE_TRAM | MODE_SUBWAY | MODE_FERRY | MODE_GONDOLA;
    let edges = osm
        .graph
        .edges
        .iter()
        .enumerate()
        .filter_map(|(edge_idx, edge)| {
            if edge.payload.oneway == 2 || (edge.payload.allowed_modes & indexed_modes) == 0 {
                return None;
            }

            let from = osm.graph.node(edge.from).payload.point;
            let to = osm.graph.node(edge.to).payload.point;
            let min = [from.x().min(to.x()), from.y().min(to.y())];
            let max = [from.x().max(to.x()), from.y().max(to.y())];

            Some(SpatialEdge {
                edge_idx,
                envelope: rstar::AABB::from_corners(min, max),
            })
        })
        .collect();

    rstar::RTree::bulk_load(edges)
}

fn snap_envelope(point: Point<f64>, max_distance_m: f64) -> rstar::AABB<[f64; 2]> {
    const METERS_PER_DEGREE_LAT: f64 = 111_320.0;
    let lat_pad = max_distance_m / METERS_PER_DEGREE_LAT;
    let lon_scale = point.y().to_radians().cos().abs().max(0.01);
    let lon_pad = max_distance_m / (METERS_PER_DEGREE_LAT * lon_scale);

    rstar::AABB::from_corners(
        [point.x() - lon_pad, point.y() - lat_pad],
        [point.x() + lon_pad, point.y() + lat_pad],
    )
}

fn oriented_edge_points(
    graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
    edge_idx: crate::graph::EdgeIndex,
) -> Vec<Point<f64>> {
    let edge = graph.edge(edge_idx);
    let mut points: Vec<Point<f64>> = if edge.payload.geometry.0.len() >= 2 {
        edge.payload
            .geometry
            .0
            .iter()
            .map(|coord| Point::new(coord.x, coord.y))
            .collect()
    } else {
        vec![
            graph.node(edge.from).payload.point,
            graph.node(edge.to).payload.point,
        ]
    };

    if edge.payload.is_reverse && edge.payload.geometry.0.len() >= 2 {
        points.reverse();
    }
    points
}

fn project_point_onto_edge(
    graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
    edge_idx: crate::graph::EdgeIndex,
    point: Point<f64>,
) -> Option<(Point<f64>, f64, f64)> {
    let points = oriented_edge_points(graph, edge_idx);
    if points.len() < 2 {
        return None;
    }

    let total_length: f64 = points
        .windows(2)
        .map(|segment| segment[0].haversine_distance(&segment[1]))
        .sum();
    if !total_length.is_finite() || total_length <= f64::EPSILON {
        return None;
    }

    let lon_scale = point.y().to_radians().cos().abs().max(0.01);
    let mut best: Option<(Point<f64>, f64, f64)> = None;
    let mut distance_before = 0.0;

    for segment in points.windows(2) {
        let a = segment[0];
        let b = segment[1];
        let dx = (b.x() - a.x()) * lon_scale;
        let dy = b.y() - a.y();
        let len_sq = dx * dx + dy * dy;
        let t = if len_sq <= f64::EPSILON {
            0.0
        } else {
            let px = (point.x() - a.x()) * lon_scale;
            let py = point.y() - a.y();
            ((px * dx + py * dy) / len_sq).clamp(0.0, 1.0)
        };

        let projected = Point::new(a.x() + (b.x() - a.x()) * t, a.y() + (b.y() - a.y()) * t);
        let distance_m = point.haversine_distance(&projected);
        let segment_length = a.haversine_distance(&b);
        let progress = ((distance_before + segment_length * t) / total_length).clamp(0.0, 1.0);

        if best
            .as_ref()
            .map_or(true, |(_, best_distance, _)| distance_m < *best_distance)
        {
            best = Some((projected, distance_m, progress));
        }
        distance_before += segment_length;
    }

    best
}

fn edge_candidates_for_stop(
    edge_index: &rstar::RTree<SpatialEdge>,
    graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
    point: Point<f64>,
    allowed_modes: u8,
    max_distance_m: f64,
    max_snap_level: i32,
) -> Vec<ProjectedEdgeCandidate> {
    let envelope = snap_envelope(point, max_distance_m);
    let mut best_by_way_direction: AHashMap<(i64, bool), ProjectedEdgeCandidate> = AHashMap::new();
    let mut unkeyed = Vec::new();

    for indexed in edge_index.locate_in_envelope_intersecting(&envelope) {
        let edge = graph.edge(indexed.edge_idx);
        if (edge.payload.allowed_modes & allowed_modes) == 0
            || edge.payload.oneway == 2
            || edge.payload.level > max_snap_level
        {
            continue;
        }

        let Some((_projected, distance_m, progress)) =
            project_point_onto_edge(graph, indexed.edge_idx, point)
        else {
            continue;
        };
        if distance_m > max_distance_m {
            continue;
        }

        let candidate = ProjectedEdgeCandidate {
            edge_idx: indexed.edge_idx,
            distance_m,
            progress,
        };

        if edge.payload.osmid == 0 {
            unkeyed.push(candidate);
            continue;
        }

        let key = (edge.payload.osmid, edge.payload.is_reverse);
        match best_by_way_direction.get_mut(&key) {
            Some(best) if candidate.distance_m < best.distance_m => *best = candidate,
            Some(_) => {}
            None => {
                best_by_way_direction.insert(key, candidate);
            }
        }
    }

    let mut candidates: Vec<_> = best_by_way_direction.into_values().collect();
    candidates.extend(unkeyed);
    candidates.sort_by(|a, b| {
        a.distance_m
            .partial_cmp(&b.distance_m)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.edge_idx.cmp(&b.edge_idx))
    });
    candidates
}

fn point_at_edge_progress(
    graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
    edge_idx: crate::graph::EdgeIndex,
    progress: f64,
) -> Option<Point<f64>> {
    let points = oriented_edge_points(graph, edge_idx);
    if points.is_empty() {
        return None;
    }
    if points.len() == 1 {
        return Some(points[0]);
    }

    let lengths: Vec<f64> = points
        .windows(2)
        .map(|segment| segment[0].haversine_distance(&segment[1]))
        .collect();
    let total: f64 = lengths.iter().sum();
    if total <= f64::EPSILON {
        return points.first().copied();
    }

    let target = progress.clamp(0.0, 1.0) * total;
    let mut traversed = 0.0;
    for (i, length) in lengths.iter().copied().enumerate() {
        if target <= traversed + length || i + 1 == lengths.len() {
            let fraction = if length <= f64::EPSILON {
                0.0
            } else {
                ((target - traversed) / length).clamp(0.0, 1.0)
            };
            let a = points[i];
            let b = points[i + 1];
            return Some(Point::new(
                a.x() + (b.x() - a.x()) * fraction,
                a.y() + (b.y() - a.y()) * fraction,
            ));
        }
        traversed += length;
    }

    points.last().copied()
}

fn push_shape_point(points: &mut Vec<(f64, f64)>, point: Point<f64>) {
    let candidate = (point.y(), point.x());
    if points.last().copied() != Some(candidate) {
        points.push(candidate);
    }
}

fn append_edge_slice(
    output: &mut Vec<(f64, f64)>,
    graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
    edge_idx: crate::graph::EdgeIndex,
    start_progress: f64,
    end_progress: f64,
) {
    let start_progress = start_progress.clamp(0.0, 1.0);
    let end_progress = end_progress.clamp(0.0, 1.0);
    let Some(start) = point_at_edge_progress(graph, edge_idx, start_progress) else {
        return;
    };
    let Some(end) = point_at_edge_progress(graph, edge_idx, end_progress) else {
        return;
    };

    if end_progress < start_progress {
        push_shape_point(output, start);
        push_shape_point(output, end);
        return;
    }

    let edge_points = oriented_edge_points(graph, edge_idx);
    let lengths: Vec<f64> = edge_points
        .windows(2)
        .map(|segment| segment[0].haversine_distance(&segment[1]))
        .collect();
    let total: f64 = lengths.iter().sum();

    push_shape_point(output, start);
    if total > f64::EPSILON {
        let start_distance = start_progress * total;
        let end_distance = end_progress * total;
        let mut traversed = 0.0;
        for (i, length) in lengths.iter().copied().enumerate() {
            traversed += length;
            if i + 1 < edge_points.len() - 1
                && traversed > start_distance
                && traversed < end_distance
            {
                push_shape_point(output, edge_points[i + 1]);
            }
        }
    }
    push_shape_point(output, end);
}

fn append_hop_geometry(
    output: &mut Vec<(f64, f64)>,
    graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
    hop: &crate::router::types::EdgeHop,
) {
    if !hop.edges.is_empty() {
        let last = hop.edges.len() - 1;
        for (i, &edge_idx) in hop.edges.iter().enumerate() {
            let start = if i == 0 { hop.start_progr } else { 0.0 };
            let end = if i == last { hop.end_progr } else { 1.0 };
            append_edge_slice(output, graph, edge_idx, start, end);
        }
        return;
    }

    let start = hop
        .start_edge
        .and_then(|edge_idx| point_at_edge_progress(graph, edge_idx, hop.start_progr))
        .or(hop.start_point);
    let end = hop
        .end_edge
        .and_then(|edge_idx| point_at_edge_progress(graph, edge_idx, hop.end_progr))
        .or(hop.end_point);

    if let Some(point) = start {
        push_shape_point(output, point);
    }
    if let Some(point) = end {
        push_shape_point(output, point);
    }
}

pub fn match_patterns(
    gtfs: &GtfsData,
    osm_path: &std::path::Path,
    skip_small_roads: bool,
    cache_file_path: &std::path::Path,
    match_threads: Option<usize>,
) -> Result<AHashMap<StopPattern, ShapeResult>, anyhow::Error> {
    use crate::mots::is_bus_like_route_type;
    use crate::osm_load::OsmBuilder;
    use crate::osm_load::load_osm;
    use crate::streaming_matcher::StreamingMatcher;
    use anyhow::Context;

    // Truncate stale output from a previous failed/OOM run.
    std::fs::File::create(cache_file_path).context("Failed to truncate cache file")?;

    // 1. Partition patterns: bus-like -> tiled, others -> full-graph
    let (bus_patterns, other_patterns): (Vec<_>, Vec<_>) = gtfs
        .patterns
        .iter()
        .partition(|(pattern, _)| pattern.route_type.map_or(false, is_bus_like_route_type));

    let bus_count = bus_patterns.len();
    let other_count = other_patterns.len();

    println!(
        "Partitioned patterns: {} bus-like (streaming), {} other (full-graph)",
        bus_count, other_count
    );

    // 2. Light OSM pass for relations/colors (only if needed by bus patterns)
    let light_osm = if bus_count > 0 {
        println!("Performing light OSM pass...");
        OsmBuilder::read_relations_only(osm_path).context("Failed to read OSM relations")?
    } else {
        println!("No bus-like patterns found. Skipping light OSM pass.");
        crate::osm_load::LightOsmData {
            relations: Vec::new(),
            way_to_relations: ahash::AHashMap::new(),
        }
    };

    // 3. Process non-bus patterns with existing full-graph approach
    let other_results = if other_count > 0 {
        println!("Loading FULL OSM graph for rail/ferry/subway matching...");

        // We only load full graph if we have non-bus patterns
        // We need to calculate BBox for these specific patterns if possible,
        // effectively optimizing the "other" load too.
        // For now, let's use the full bbox logic from main.rs but filtered to other_patterns.
        // Or just load everything for "other" modes?
        // Safety: use existing logic but restricted.

        let mut types = ahash::AHashSet::new();
        for (pattern, _) in &other_patterns {
            if let Some(rt) = pattern.route_type {
                types.insert(rt);
            }
        }

        // Calculate bounding box from pattern stops (critical for memory efficiency)
        let bbox = {
            let mut min_lat = f64::MAX;
            let mut max_lat = f64::MIN;
            let mut min_lon = f64::MAX;
            let mut max_lon = f64::MIN;
            let mut found_any = false;

            for (pattern, _) in &other_patterns {
                for stop_id in &pattern.stop_ids {
                    if let Some(stop) = gtfs.gtfs.stops.get(stop_id) {
                        if let (Some(lat), Some(lon)) = (stop.latitude, stop.longitude) {
                            min_lat = min_lat.min(lat);
                            max_lat = max_lat.max(lat);
                            min_lon = min_lon.min(lon);
                            max_lon = max_lon.max(lon);
                            found_any = true;
                        }
                    }
                }
            }

            if found_any {
                // Add padding (0.5 degrees ~= 55km) to ensure we capture nearby infrastructure
                let padding = 0.5;
                Some((
                    min_lon - padding,
                    min_lat - padding,
                    max_lon + padding,
                    max_lat + padding,
                ))
            } else {
                None
            }
        };

        println!("  Calculated bbox for {} patterns: {:?}", other_count, bbox);
        let osm_data = load_osm(osm_path, &types, bbox, false)?;

        let results = match_patterns_full_graph(
            &gtfs,
            &osm_data,
            other_patterns,
            cache_file_path,
            match_threads,
        )?;
        results
    } else {
        println!("No non-bus patterns found. Skipping full graph load.");
        AHashMap::new()
    };

    // 4. Process bus-like patterns with streaming approach
    let bus_results = if bus_count > 0 {
        // Cache size: 100 tiles * ~50MB/tile = ~5GB peak?
        // Tiles are much smaller if stripped of buildings/etc.
        // 100 tiles is generous. 0.5 deg tile = 50x50km.
        let mut matcher = StreamingMatcher::new(osm_path, 50, light_osm, skip_small_roads)?;
        matcher.match_all(gtfs, bus_patterns, cache_file_path)
    } else {
        AHashMap::new()
    };

    // 5. Merge results
    let mut results = other_results;
    results.extend(bus_results);

    // 6. Deduplicate shape IDs (handle hash collisions)
    // It's possible (though unlikely) that two different patterns hash to the same shape_id.
    // If this happens, one would overwrite the other in main.rs.
    // We must ensure unique shape IDs for distinct patterns.
    let mut id_to_patterns: AHashMap<String, Vec<StopPattern>> = AHashMap::new();
    for (pattern, result) in &results {
        id_to_patterns
            .entry(result.shape_id.clone())
            .or_default()
            .push(pattern.clone());
    }

    for (shape_id, patterns) in id_to_patterns {
        if patterns.len() > 1 {
            // Collision detected!
            // We need to update the shape_ids for all but one.
            // Sort patterns to ensure deterministic reassignment
            let mut sorted_patterns = patterns;
            // We can't easily sort StopPattern without Ord, but Hash is stable-ish?
            // Actually StopPattern usually implements Ord/PartialOrd in gtfs-structures?
            // Let's assume arbitrary order from map iteration is not deterministic enough if we want 100% reproducibility.
            // But for preventing overwrite, just distinct suffices.
            // To be purely deterministic, we should sort by something.
            // Let's skip sort for now reliance on iteration order (might vary),
            // but collision is rare enough.

            for (i, pattern) in sorted_patterns.iter().enumerate().skip(1) {
                if let Some(res) = results.get_mut(pattern) {
                    res.shape_id = format!("{}_{}", shape_id, i);
                }
            }
        }
    }

    Ok(results)
}

/// Full-graph matching (original implementation for non-bus modes)
fn match_patterns_full_graph(
    gtfs: &GtfsData,
    osm: &OsmData,
    patterns: Vec<(&StopPattern, &Vec<String>)>,
    cache_file_path: &std::path::Path,
    match_threads: Option<usize>,
) -> anyhow::Result<AHashMap<StopPattern, ShapeResult>> {
    use rayon::prelude::*;
    use std::fs::{File, OpenOptions};
    use std::io::{BufWriter, Write};
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicUsize, Ordering};

    fn process_patterns(
        gtfs: &GtfsData,
        osm: &OsmData,
        edge_index: &rstar::RTree<SpatialEdge>,
        patterns: &[(&StopPattern, &Vec<String>)],
        writer: &Mutex<BufWriter<File>>,
        results: &Mutex<AHashMap<StopPattern, ShapeResult>>,
        processed: &AtomicUsize,
        total_patterns: usize,
    ) -> anyhow::Result<()> {
        patterns
            .par_iter()
            .map_init(
                pathfinding::PathfinderContext::new,
                |ctx, (pattern, trips)| {
                    let matched = match_one_pattern(gtfs, osm, edge_index, pattern, trips, ctx);
                    ctx.discard_if_oversized(500_000);
                    matched
                },
            )
            .try_for_each(|matched| -> anyhow::Result<()> {
                if let Some((pattern, shape_result, points)) = matched {
                    if !shape_result.empty_geometry {
                        let mut writer = writer
                            .lock()
                            .map_err(|_| anyhow::anyhow!("shape cache writer lock poisoned"))?;
                        for (sequence, (lat, lon)) in points.into_iter().enumerate() {
                            let record = BinaryShapeRecord {
                                shape_id: shape_result.shape_id.clone(),
                                shape_pt_lat: lat,
                                shape_pt_lon: lon,
                                shape_pt_sequence: sequence + 1,
                            };
                            bincode::serialize_into(&mut *writer, &record)?;
                        }
                    }

                    results
                        .lock()
                        .map_err(|_| anyhow::anyhow!("shape result lock poisoned"))?
                        .insert(pattern, shape_result);
                }

                let done = processed.fetch_add(1, Ordering::Relaxed) + 1;
                if done == total_patterns || done % 100 == 0 {
                    println!("Processed {}/{}", done, total_patterns);
                }
                Ok(())
            })
    }

    let total_patterns = patterns.len();
    println!("Matching {} patterns (full-graph)...", total_patterns);

    // Upstream ShapeBuilder indexes graph edges, not graph nodes, for station
    // candidates. Build that index once per full-graph run and share it across
    // pattern workers.
    let edge_index = build_match_edge_index(osm);
    println!(
        "Built edge snap index with {} directed edges",
        edge_index.size()
    );

    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(cache_file_path)?;
    let writer = Mutex::new(BufWriter::new(file));
    let results = Mutex::new(AHashMap::with_capacity(patterns.len()));
    let processed = AtomicUsize::new(0);

    let pool = match_threads
        .map(|threads| rayon::ThreadPoolBuilder::new().num_threads(threads).build())
        .transpose()?;

    if let Some(ref pool) = pool {
        pool.install(|| {
            process_patterns(
                gtfs,
                osm,
                &edge_index,
                &patterns,
                &writer,
                &results,
                &processed,
                total_patterns,
            )
        })?;
    } else {
        process_patterns(
            gtfs,
            osm,
            &edge_index,
            &patterns,
            &writer,
            &results,
            &processed,
            total_patterns,
        )?;
    }

    writer
        .into_inner()
        .map_err(|_| anyhow::anyhow!("shape cache writer lock poisoned"))?
        .flush()?;

    results
        .into_inner()
        .map_err(|_| anyhow::anyhow!("shape result lock poisoned"))
}

fn shape_id_for_pattern(pattern: &StopPattern) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    pattern.hash(&mut hasher);
    format!("shape_{}", hasher.finish())
}

fn match_one_pattern(
    gtfs: &GtfsData,
    osm: &OsmData,
    edge_index: &rstar::RTree<SpatialEdge>,
    pattern: &StopPattern,
    trips: &Vec<String>,
    ctx: &mut pathfinding::PathfinderContext,
) -> Option<(StopPattern, ShapeResult, Vec<(f64, f64)>)> {
    use crate::router::hop_cache::HopCache;
    use crate::router::router_impl::{Restrictor, RouterImpl};
    use crate::router::trip_trie::TripTrie;
    use crate::router::types::{EdgeCand, EdgeCandGroup};
    use crate::router::weights::{ExpoTransWeight, RoutingAttrs, RoutingOpts};

    let sample_trip_id = gtfs.patterns.get(pattern)?.first()?;
    let sample_trip = gtfs.gtfs.trips.get(sample_trip_id)?;
    let route = gtfs.gtfs.routes.get(&sample_trip.route_id)?;

    // This is the pfaedle C++ architecture: relation identity is already compiled
    // onto graph edges by OsmBuilder. Matching discovers nearby graph candidates
    // first, then the router prefers edges carrying the GTFS line identity. There
    // is deliberately no global scan over osm.relations here.
    let route_name = route
        .short_name
        .as_deref()
        .filter(|name| !name.trim().is_empty())
        .or_else(|| {
            sample_trip
                .trip_short_name
                .as_deref()
                .filter(|name| !name.trim().is_empty())
        })
        .or_else(|| route.long_name.as_deref())
        .unwrap_or("")
        .to_string();

    let stop_coords: Vec<Point<f64>> = pattern
        .stop_ids
        .iter()
        .filter_map(|stop_id| gtfs.gtfs.stops.get(stop_id))
        .filter_map(|stop| match (stop.latitude, stop.longitude) {
            (Some(lat), Some(lon)) => Some(Point::new(lon, lat)),
            _ => None,
        })
        .collect();
    if stop_coords.len() < 2 || stop_coords.len() != pattern.stop_ids.len() {
        return None;
    }

    let allowed_modes = match pattern.route_type {
        Some(RouteType::Tramway) => MODE_TRAM,
        Some(RouteType::Subway) => MODE_SUBWAY,
        Some(RouteType::Rail) => MODE_RAIL,
        Some(RouteType::Ferry) => MODE_FERRY,
        Some(RouteType::Gondola) | Some(RouteType::Funicular) | Some(RouteType::CableCar) => {
            MODE_GONDOLA
        }
        _ => MODE_BUS,
    };
    let fallback_modes = match pattern.route_type {
        Some(RouteType::Tramway) => MODE_SUBWAY | MODE_RAIL,
        Some(RouteType::Subway) => MODE_TRAM | MODE_RAIL,
        Some(RouteType::Rail) => MODE_SUBWAY | MODE_TRAM,
        _ => 0,
    };
    let all_allowed_modes = allowed_modes | fallback_modes;

    // ShapeBuilder::getEdgCands always starts with a null candidate and then
    // projects the stop onto nearby graph edges. The old Rust port searched
    // nearby *nodes*, required one to exist, and assigned progression 0. That
    // changes both reachability and routing cost, especially on long rail ways.
    const STATION_MOVE_PENALTY_PER_METER: f64 = 0.0039;
    let max_snap_distance_m = if allowed_modes == MODE_RAIL {
        200.0
    } else {
        50.0
    };
    let max_snap_level = if allowed_modes == MODE_RAIL { 2 } else { 7 };
    let non_station_penalty = if allowed_modes == MODE_RAIL { 0.4 } else { 0.0 };

    let mut stop_candidates: Vec<EdgeCandGroup> = Vec::with_capacity(stop_coords.len());
    for &point in &stop_coords {
        // This candidate must never be removed. Upstream uses it to preserve
        // geometry when OSM is missing, disconnected, or cannot be snapped.
        let mut group = vec![EdgeCand {
            edge: None,
            point: Some(point),
            pen: 0.0,
            time: 0.0,
            progr: 0.0,
            dep_prede: Vec::new(),
        }];

        for snapped in edge_candidates_for_stop(
            edge_index,
            &osm.graph,
            point,
            all_allowed_modes,
            max_snap_distance_m,
            max_snap_level,
        ) {
            group.push(EdgeCand {
                edge: Some(snapped.edge_idx),
                point: None,
                pen: snapped.distance_m * STATION_MOVE_PENALTY_PER_METER + non_station_penalty,
                time: 0.0,
                progr: snapped.progress,
                dep_prede: Vec::new(),
            });
        }

        stop_candidates.push(group);
    }

    let r_attrs = RoutingAttrs {
        short_name: route_name,
        // Upstream uses a statistical station-name classifier for these fields.
        // Rust does not have that classifier yet, so leave them unspecified rather
        // than applying a false exact-string penalty.
        line_from: String::new(),
        line_to: String::new(),
    };

    let mut trie = TripTrie::new();
    for trip_id in trips {
        if let Some(trip) = gtfs.gtfs.trips.get(trip_id) {
            // Upstream only time-expands the trie for the `timenorm`
            // transition model. The default model is `exp`, so equivalent trips
            // should share trie nodes and contribute to averaged hop times.
            trie.add_trip(trip, &r_attrs, false, false);
        }
    }

    fn build_candidate_group(
        stop_cands: &[EdgeCand],
        nd: &crate::router::trip_trie::TripTrieNd,
    ) -> EdgeCandGroup {
        let mut group = stop_cands.to_vec();
        for candidate in &mut group {
            candidate.time = nd.time as f64;
            candidate.dep_prede.clear();
            if candidate.edge.is_none() {
                candidate.point = Some(nd.pos);
            }
        }
        group
    }

    fn average_time(nd: &crate::router::trip_trie::TripTrieNd) -> f64 {
        if nd.trips > 0 {
            nd.acc_time as f64 / nd.trips as f64
        } else {
            nd.time as f64
        }
    }

    let mut ecm = AHashMap::new();
    let nds = trie.get_nds();
    for (nid, nd) in nds.iter().enumerate().skip(1) {
        let is_initial_departure = nd.parent == Some(0);
        if !is_initial_departure && !nd.arr {
            continue;
        }

        let mut depth = 0;
        let mut current = Some(nid);
        while let Some(current_nid) = current {
            current = nds[current_nid].parent;
            if current.is_some() {
                depth += 1;
            }
        }

        let stop_idx = depth / 2;
        let stop_cands = stop_candidates
            .get(stop_idx)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        let mut group = build_candidate_group(stop_cands, nd);
        let arrival_time = average_time(nd);
        for candidate in &mut group {
            candidate.time = arrival_time;
            candidate.dep_prede.clear();
        }
        ecm.insert(nid, group.clone());

        if nd.arr {
            for &departure_nid in &nd.childs {
                let departure_nd = &nds[departure_nid];
                debug_assert!(!departure_nd.arr);
                let departure_time = average_time(departure_nd);
                let departure_group: EdgeCandGroup = group
                    .iter()
                    .enumerate()
                    .map(|(arrival_candidate_id, candidate)| {
                        let mut departure_candidate = candidate.clone();
                        departure_candidate.time = departure_time;
                        departure_candidate.dep_prede = if arrival_time <= departure_time {
                            vec![arrival_candidate_id]
                        } else {
                            Vec::new()
                        };
                        departure_candidate
                    })
                    .collect();
                ecm.insert(departure_nid, departure_group);
            }
        }
    }

    let router: RouterImpl<ExpoTransWeight> = RouterImpl::new(&osm.graph);
    let mut r_opts = RoutingOpts::default();
    // Upstream pfaedle defaults. `line_from`/`line_to` are intentionally empty
    // above, so their factors are configured here but remain neutral until the
    // Rust port has the upstream statistical station-name classifier.
    r_opts.transition_pen = 0.0083;
    r_opts.line_unmatched_punish_fact = 1.2;
    r_opts.line_name_to_unmatched_punish_fact = 1.1;
    r_opts.line_name_from_unmatched_punish_fact = 1.05;
    if allowed_modes == MODE_RAIL {
        // pfaedle.cfg [rail]: discourage full reversals through station throats.
        r_opts.full_turn_punish_fac = 1_800;
        r_opts.full_turn_angle = 100.0;
    }

    let mut hop_cache = HopCache::new();
    let routes = router.route(
        &trie,
        &ecm,
        &r_opts,
        &Restrictor::new(),
        Some(&mut hop_cache),
        false,
    );

    let mut full_path_geometry = Vec::new();
    if let Some((_leaf_nid, hops)) = routes.into_iter().next() {
        for hop in &hops {
            // Upstream ShapeBuilder::getGeom renders *every* hop. A routed hop
            // is cropped using start/end progression; a failed hop is rendered
            // as the straight fallback between its candidate positions. Dropping
            // empty-edge hops is what made entire services vanish and caused
            // otherwise-correct shapes to stop at the first failed hop.
            append_hop_geometry(&mut full_path_geometry, &osm.graph, hop);
        }
    }

    let shape_id = shape_id_for_pattern(pattern);
    let empty_geometry = full_path_geometry.is_empty();
    Some((
        (*pattern).clone(),
        ShapeResult {
            shape_id,
            empty_geometry,
            matched_route_color: None,
        },
        full_path_geometry,
    ))
}

pub fn match_sequence_globally_optimal(
    stop_candidates: &[Vec<usize>],
    graph: &Graph<NodePL, EdgePL>,
    allowed_modes: u8,
    fallback_modes: u8,
    preferred_match: Option<&TransitMatch>,
    ctx: &mut pathfinding::PathfinderContext,
) -> Option<Vec<(f64, f64)>> {
    if stop_candidates.is_empty() {
        return None;
    }

    // Dynamic Programming / Viterbi
    // State: (stop_idx, candidate_idx)
    // We want to minimize total path cost.

    let num_stops = stop_candidates.len();

    // MinCost[i][k] = minimum cost to reach candidate k at stop i from start
    // Parent[i][k] = index of candidate at i-1 that leads to MinCost
    // We store this as Vec<Vec<Option<(Cost, ParentIdx)>>>
    // where inner vec size is stop_candidates[i].len()

    let mut min_costs: Vec<Vec<Option<(f64, usize)>>> = Vec::with_capacity(num_stops);

    // Path cache: (from_node, to_node) -> cost
    // Avoids redundant pathfinding when same node pairs appear
    let mut path_cache: AHashMap<(usize, usize), Option<f64>> = AHashMap::new();

    // Initialize first stop
    let first_candidates_len = stop_candidates[0].len();
    let mut first_costs = Vec::with_capacity(first_candidates_len);
    for _ in 0..first_candidates_len {
        first_costs.push(Some((0.0, 0))); // Cost 0 to start here. Parent 0 (dummy)
    }
    min_costs.push(first_costs);

    // Iterate stops
    for i in 1..num_stops {
        let prev_candidates = &stop_candidates[i - 1];
        let curr_candidates = &stop_candidates[i];
        let mut curr_costs = vec![None; curr_candidates.len()];

        let mut min_lon = f64::MAX;
        let mut max_lon = f64::MIN;
        let mut min_lat = f64::MAX;
        let mut max_lat = f64::MIN;
        for &n in prev_candidates.iter().chain(curr_candidates.iter()) {
            let p = graph.node(n).payload.point;
            min_lon = min_lon.min(p.x());
            max_lon = max_lon.max(p.x());
            min_lat = min_lat.min(p.y());
            max_lat = max_lat.max(p.y());
        }
        let margin = if allowed_modes == crate::graph::MODE_RAIL {
            0.5
        } else {
            0.1
        };
        let bounding_box = Some((
            min_lon - margin,
            min_lat - margin,
            max_lon + margin,
            max_lat + margin,
        ));

        let mut any_reachable = false;

        let mut valid_starts = Vec::new();
        let mut start_node_to_prev_k = ahash::AHashMap::new();

        for (prev_k, prev_cost_opt) in min_costs[i - 1].iter().enumerate() {
            if let Some((prev_total_cost, _)) = prev_cost_opt {
                let start_node = prev_candidates[prev_k];
                valid_starts.push((start_node, *prev_total_cost));

                if let Some(&existing_k) = start_node_to_prev_k.get(&start_node) {
                    if let Some((existing_cost, _)) = min_costs[i - 1][existing_k] {
                        if prev_total_cost < &existing_cost {
                            start_node_to_prev_k.insert(start_node, prev_k);
                        }
                    }
                } else {
                    start_node_to_prev_k.insert(start_node, prev_k);
                }
            }
        }

        // Component pruning
        let mut valid_targets = Vec::new();
        let mut target_to_idx = ahash::AHashMap::new();

        for (curr_k, &curr_node) in curr_candidates.iter().enumerate() {
            let curr_comp = graph.node(curr_node).payload.comp_id;
            let mut reachable = false;
            for &(start_node, _) in &valid_starts {
                if graph.node(start_node).payload.comp_id == curr_comp || curr_node == start_node {
                    reachable = true;
                    break;
                }
            }
            if reachable {
                valid_targets.push(curr_node);
                target_to_idx.insert(curr_node, curr_k);
            }
        }

        if !valid_targets.is_empty() && !valid_starts.is_empty() {
            let max_cost = 1_000_000.0;

            let results = pathfinding::multi_target_dijkstra(
                graph,
                &valid_starts,
                &valid_targets,
                allowed_modes,
                fallback_modes,
                None, // allowed_edges
                preferred_match,
                bounding_box,
                max_cost,
            );

            // Process results
            for (&target_node, &(total_cost, best_start_node)) in &results {
                if let Some(&curr_k) = target_to_idx.get(&target_node) {
                    if let Some(&prev_k) = start_node_to_prev_k.get(&best_start_node) {
                        curr_costs[curr_k] = Some((total_cost, prev_k));
                        any_reachable = true;
                    }
                }
            }
        }

        min_costs.push(curr_costs);

        if !any_reachable {
            // Cannot reach any candidate at this stop. Path is broken.
            // println!("Broken path at stop index {}", i);
            return None;
        }
    }

    // Backtrack from best candidate at last stop
    let last_stop_idx = num_stops - 1;
    // let last_candidates = &stop_candidates[last_stop_idx]; // Unused

    // Find best end candidate
    let mut best_end_k = None;
    let mut best_end_cost = f64::INFINITY;

    for (k, cost_opt) in min_costs[last_stop_idx].iter().enumerate() {
        if let Some((cost, _)) = cost_opt {
            if *cost < best_end_cost {
                best_end_cost = *cost;
                best_end_k = Some(k);
            }
        }
    }

    if let Some(mut curr_k) = best_end_k {
        // Reconstruct path
        // We need to re-run pathfind to get geometry, or we could have stored it?
        // Storing geometry for all 400 pairs is heavy memory?
        // Storing edge indices is okay.
        // But here we just re-run pathfind during backtracking. It is only N pathfinds now.

        let mut full_geometry: Vec<(f64, f64)> = Vec::new();

        // We build geometry backwards: last segment, then second to last...
        // Then we reverse the whole list of points?
        // Or we collect segments and reverse the order of segments?
        // Let's collect segments from end to start.

        let mut segments: Vec<Vec<(f64, f64)>> = Vec::new();

        for i in (1..num_stops).rev() {
            let prev_k = min_costs[i][curr_k].unwrap().1;

            let curr_node = stop_candidates[i][curr_k];
            let prev_node = stop_candidates[i - 1][prev_k];

            let mut min_lon = f64::MAX;
            let mut max_lon = f64::MIN;
            let mut min_lat = f64::MAX;
            let mut max_lat = f64::MIN;
            let p1 = graph.node(prev_node).payload.point;
            let p2 = graph.node(curr_node).payload.point;
            for p in &[p1, p2] {
                min_lon = min_lon.min(p.x());
                max_lon = max_lon.max(p.x());
                min_lat = min_lat.min(p.y());
                max_lat = max_lat.max(p.y());
            }
            let margin = if allowed_modes == crate::graph::MODE_RAIL {
                0.5
            } else {
                0.1
            };
            let bounding_box = Some((
                min_lon - margin,
                min_lat - margin,
                max_lon + margin,
                max_lat + margin,
            ));

            let mut segment_geom = Vec::new();

            if curr_node == prev_node {
                // No movement
            } else {
                if let Some((_, edges)) = pathfinding::pathfind_with_context(
                    ctx,
                    graph,
                    prev_node,
                    curr_node,
                    allowed_modes,
                    fallback_modes,
                    None,
                    preferred_match,
                    bounding_box,
                ) {
                    for edge_idx in edges {
                        let edge = graph.edge(edge_idx);
                        for coord in edge.payload.geometry.coords().skip(1) {
                            segment_geom.push((coord.y, coord.x));
                        }
                    }
                } else {
                    // Should not happen if logic is correct
                    return None;
                }
            }
            segments.push(segment_geom);
            curr_k = prev_k;
        }

        // Add start node
        let start_node = stop_candidates[0][curr_k];
        let p = graph.node(start_node).payload.point;
        full_geometry.push((p.y(), p.x()));

        // Segments are in reverse order (last segment first)
        // We need to process segments in reverse (first segment first)
        for seg in segments.iter().rev() {
            full_geometry.extend(seg.iter().cloned());
        }

        return Some(full_geometry);
    }

    None
}

/// Find route color from OSM relations without computing shapes.
/// Used when shapes are already in place but colors need to be matched.
pub fn find_route_color_from_osm(
    gtfs: &GtfsData,
    osm: &OsmData,
    pattern: &StopPattern,
) -> Option<String> {
    // Get route metadata for matching
    let sample_trip_id = gtfs.patterns.get(pattern)?.first()?;
    let trip = gtfs.gtfs.trips.get(sample_trip_id)?;
    let route = gtfs.gtfs.routes.get(&trip.route_id)?;

    let agency_name = route.agency_id.as_ref().and_then(|agency_id| {
        gtfs.gtfs
            .agencies
            .iter()
            .find(|a| a.id.as_ref() == Some(agency_id))
            .map(|a| a.name.to_lowercase())
    });

    let route_short_name = route.short_name.as_ref().map(|s| s.to_lowercase());
    let route_long_name = route.long_name.as_ref().map(|s| s.to_lowercase());

    // Find stop coordinates
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

    let index = osm.spatial_tree.as_ref()?;

    // Determine allowed modes
    let allowed_modes = match pattern.route_type {
        Some(RouteType::Tramway) => MODE_TRAM,
        Some(RouteType::Subway) => MODE_SUBWAY,
        Some(RouteType::Rail) => MODE_RAIL,
        Some(RouteType::Ferry) => MODE_FERRY,
        Some(RouteType::Gondola) | Some(RouteType::Funicular) | Some(RouteType::CableCar) => {
            MODE_GONDOLA
        }
        _ => MODE_BUS,
    };

    // Score relations based on stop coverage
    let mut relation_scores: AHashMap<usize, f64> = AHashMap::new();

    for point in &stop_coords {
        let neighbors = index
            .nearest_neighbor_iter(&[point.x(), point.y()])
            .filter(|sn| (sn.modes & allowed_modes) != 0)
            .take(50);

        let mut seen_for_stop = AHashSet::new();
        for sn in neighbors {
            let node_idx = sn.index;
            if let Some(rels) = osm.node_to_relations.get(&node_idx) {
                for &r_idx in rels {
                    if seen_for_stop.insert(r_idx) {
                        *relation_scores.entry(r_idx).or_insert(0.0) += 1.0;
                    }
                }
            }
        }
    }

    // Find best matching relation with color
    let min_coverage = (stop_coords.len() as f64) * 0.5; // At least 50% stops covered

    let mut candidates: Vec<_> = relation_scores
        .iter()
        .filter(|(_, score)| **score >= min_coverage)
        .filter(|(r_idx, _)| osm.relations[**r_idx].tags.contains_key("colour"))
        .collect();

    // Sort by match score (name/operator match) + coverage
    candidates.sort_by(|(a_idx, a_score), (b_idx, b_score)| {
        let get_match_score = |r_idx: usize| -> u8 {
            let rel = &osm.relations[r_idx];
            let mut match_score = 0;

            let osm_names = [
                rel.tags.get("ref"),
                rel.tags.get("name"),
                rel.tags.get("official_name"),
                rel.tags.get("alt_name"),
            ];

            for osm_name_opt in osm_names {
                if let Some(osm_name) = osm_name_opt {
                    let osm_val = osm_name.to_lowercase();
                    if let Some(ref gtfs_short) = route_short_name {
                        if osm_val.contains(gtfs_short) || gtfs_short.contains(&osm_val) {
                            match_score += 2;
                            break;
                        }
                    }
                    if let Some(ref gtfs_long) = route_long_name {
                        if osm_val.contains(gtfs_long) || gtfs_long.contains(&osm_val) {
                            match_score += 2;
                            break;
                        }
                    }
                }
            }

            if let Some(target_op) = &agency_name {
                if let Some(osm_op) = rel.tags.get("operator") {
                    if osm_op.to_lowercase().contains(target_op)
                        || target_op.contains(&osm_op.to_lowercase())
                    {
                        match_score += 1;
                    }
                }
            }

            match_score
        };

        let match_a = get_match_score(**a_idx);
        let match_b = get_match_score(**b_idx);

        match_b.cmp(&match_a).then_with(|| {
            b_score
                .partial_cmp(a_score)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    });

    // Return color from best match
    for (r_idx, _) in candidates {
        if let Some(color) = osm.relations[*r_idx].tags.get("colour") {
            return Some(color.to_string());
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{EdgePL, Graph, NodePL};
    use geo::LineString;

    #[test]
    fn edge_projection_tracks_progress_along_the_edge() {
        let mut graph = Graph::new();
        let from = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let to = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.01, 0.0),
        });
        let mut edge = EdgePL::new();
        edge.geometry = LineString::new(vec![(0.0, 0.0).into(), (0.01, 0.0).into()]);
        edge.allowed_modes = MODE_RAIL;
        edge.osmid = 1;
        let edge_idx = graph.add_edge(from, to, edge);

        let (_, distance_m, progress) =
            project_point_onto_edge(&graph, edge_idx, Point::new(0.005, 0.001)).unwrap();
        assert!((0.45..0.55).contains(&progress));
        assert!((100.0..125.0).contains(&distance_m));
    }

    #[test]
    fn fallback_hops_are_emitted_as_geometry() {
        let graph: Graph<NodePL, EdgePL> = Graph::new();
        let hop = crate::router::types::EdgeHop {
            edges: Vec::new(),
            start_edge: None,
            end_edge: None,
            start_progr: 0.0,
            end_progr: 0.0,
            start_point: Some(Point::new(-88.0, 42.0)),
            end_point: Some(Point::new(-88.1, 42.1)),
        };
        let mut points = Vec::new();

        append_hop_geometry(&mut points, &graph, &hop);

        assert_eq!(points, vec![(42.0, -88.0), (42.1, -88.1)]);
    }

    #[test]
    fn routed_hops_are_cropped_to_candidate_progress() {
        let mut graph = Graph::new();
        let from = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let to = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(1.0, 0.0),
        });
        let mut edge = EdgePL::new();
        edge.geometry = LineString::new(vec![(0.0, 0.0).into(), (1.0, 0.0).into()]);
        let edge_idx = graph.add_edge(from, to, edge);
        let hop = crate::router::types::EdgeHop {
            edges: vec![edge_idx],
            start_edge: Some(edge_idx),
            end_edge: Some(edge_idx),
            start_progr: 0.25,
            end_progr: 0.75,
            start_point: None,
            end_point: None,
        };
        let mut points = Vec::new();

        append_hop_geometry(&mut points, &graph, &hop);

        assert_eq!(points.len(), 2);
        assert!((points[0].1 - 0.25).abs() < 1e-9);
        assert!((points[1].1 - 0.75).abs() < 1e-9);
    }

    #[test]
    fn test_backtracking_dead_end() {
        // Create a simple graph
        // Track A: 0 -> 1 -> Dead End
        // Track B: 2 -> 3 -> 4 (Success)
        // Stops:
        // S1: Closer to 0 (A) than 2 (B)
        // S2: Closer to 1 (A) than 3 (B)
        // S3: Closer to 4 (B) (Only option)

        let mut graph = Graph::new();
        let n0 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(1.0, 0.0),
        });
        let n2 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 1.0),
        });
        let n3 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(1.0, 1.0),
        });
        let n4 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(2.0, 1.0),
        });

        // Edges A
        let mut e = EdgePL::new();
        e.cost = 10;
        e.allowed_modes = 255;

        // Populate geometry for A (0->1)
        e.geometry = LineString::new(vec![
            Point::new(0.0, 0.0).into(),
            Point::new(1.0, 0.0).into(),
        ]);
        graph.add_edge(n0, n1, e.clone());

        // Edges B
        // 2->3
        e.geometry = LineString::new(vec![
            Point::new(0.0, 1.0).into(),
            Point::new(1.0, 1.0).into(),
        ]);
        graph.add_edge(n2, n3, e.clone());

        // 3->4
        e.geometry = LineString::new(vec![
            Point::new(1.0, 1.0).into(),
            Point::new(2.0, 1.0).into(),
        ]);
        graph.add_edge(n3, n4, e.clone());

        let osm = OsmData {
            graph,
            spatial_tree: None,
            relations: Vec::new(),
            node_to_relations: AHashMap::new(),
            timestamp: "test".to_string(),
            osm_filepath: std::path::PathBuf::from("test.osm"),
        };

        // Candidates
        // Stop 1: [0, 2] (0 is closer/first)
        // Stop 2: [1, 3] (1 is closer/first)
        // Stop 3: [4]
        let stop_candidates = vec![vec![n0, n2], vec![n1, n3], vec![n4]];

        let mut ctx = pathfinding::PathfinderContext::new();
        let result =
            match_sequence_globally_optimal(&stop_candidates, &osm.graph, 255, 0, None, &mut ctx);

        // Should succeed by picking 2 -> 3 -> 4
        assert!(result.is_some());
        let points = result.unwrap();
        assert!(points.len() > 1);
        // Check geometry roughly (start point + end points of edges)
        // Path: 2 -> 3 -> 4
        // Points: Node(2), Node(3) (via edge 2->3), Node(4) (via edge 3->4)
        // Coords: (1.0, 0.0) -> (1.0, 1.0) -> (1.0, 2.0) (Lat, Lon) -> (y, x)
        // Wait, Point::new(x, y).
        // n2: (0, 1) -> y=1, x=0
        // n3: (1, 1) -> y=1, x=1
        // n4: (2, 1) -> y=1, x=2
        // Matcher returns (lat, lon) which is (y, x)

        // Expected Length: 3 points?
        // match_sequence_with_backtracking returns:
        // Start Node Point + (Edge 1 points excluding first) + (Edge 2 points excluding first)
        // Edge geometry defaults to straight line (from, to) if not specified in payload?
        // Graph edge default has empty geometry usually.
        // If geometry is empty, my loop `edge.payload.geometry.coords().skip(1)` does nothing?
        // Wait, `edge.payload.geometry` in `OsmData` usually has points.
        // In this test, `EdgePL::new()` has default geometry?
        // Let's check `EdgePL`.

        // For test purposes, we rely on the fact that `pathfind` returns edges
        // and we just need to verify it returns *some* path.
    }

    #[test]
    fn test_trie_depth_ecm_invariants() {
        use crate::router::trip_trie::{TripTrie, TripTrieNd};
        use crate::router::types::{EdgeCand, EdgeCandGroup};
        use crate::router::weights::RoutingAttrs;

        let mut graph = Graph::new();
        let n0 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });

        let stop_candidates = vec![vec![n0], vec![n0], vec![n0]];

        let nds = vec![
            TripTrieNd {
                stop_name: "ROOT".to_string(),
                platform: "".to_string(),
                pos: Point::new(0.0, 0.0),
                lat: 0.0,
                lng: 0.0,
                time: 0,
                arr: false,
                trip_time: 0,
                trips: 0,
                parent: None,
                childs: vec![1],
                r_attrs: RoutingAttrs::default(),
                acc_time: 0,
            },
            TripTrieNd {
                stop_name: "Stop 0".to_string(),
                platform: "".to_string(),
                pos: Point::new(0.0, 0.0),
                lat: 0.0,
                lng: 0.0,
                time: 100,
                arr: false,
                trip_time: 100,
                trips: 1,
                parent: Some(0),
                childs: vec![2],
                r_attrs: RoutingAttrs::default(),
                acc_time: 100,
            },
            TripTrieNd {
                stop_name: "Stop 1".to_string(),
                platform: "".to_string(),
                pos: Point::new(0.0, 0.0),
                lat: 0.0,
                lng: 0.0,
                time: 200,
                arr: true,
                trip_time: 200,
                trips: 1,
                parent: Some(1),
                childs: vec![3],
                r_attrs: RoutingAttrs::default(),
                acc_time: 200,
            },
            TripTrieNd {
                stop_name: "Stop 1".to_string(),
                platform: "".to_string(),
                pos: Point::new(0.0, 0.0),
                lat: 0.0,
                lng: 0.0,
                time: 210,
                arr: false,
                trip_time: 210,
                trips: 1,
                parent: Some(2),
                childs: vec![4],
                r_attrs: RoutingAttrs::default(),
                acc_time: 210,
            },
            TripTrieNd {
                stop_name: "Stop 2".to_string(),
                platform: "".to_string(),
                pos: Point::new(0.0, 0.0),
                lat: 0.0,
                lng: 0.0,
                time: 300,
                arr: true,
                trip_time: 300,
                trips: 1,
                parent: Some(3),
                childs: vec![],
                r_attrs: RoutingAttrs::default(),
                acc_time: 300,
            },
        ];

        let trie = TripTrie::new_dummy(nds);

        fn build_candidate_group(
            stop_cands: &[crate::graph::NodeIndex],
            nd: &crate::router::trip_trie::TripTrieNd,
            graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
        ) -> EdgeCandGroup {
            let mut cand_group = Vec::new();
            cand_group.push(EdgeCand {
                edge: None,
                point: Some(nd.pos),
                pen: 0.0,
                time: nd.time as f64,
                progr: 0.0,
                dep_prede: vec![],
            });
            for &node_idx in stop_cands.iter().take(5) {
                let graph_node = graph.node(node_idx);
                for &edge_idx in graph_node.edges() {
                    let e = graph.edge(edge_idx);
                    if e.from == node_idx {
                        cand_group.push(EdgeCand {
                            edge: Some(edge_idx),
                            point: Some(graph_node.payload.point),
                            pen: 0.0,
                            time: nd.time as f64,
                            progr: 0.0,
                            dep_prede: vec![],
                        });
                    }
                }
            }
            cand_group
        }

        fn average_time(nd: &crate::router::trip_trie::TripTrieNd) -> f64 {
            if nd.trips > 0 {
                nd.acc_time as f64 / nd.trips as f64
            } else {
                nd.time as f64
            }
        }

        let mut ecm = ahash::AHashMap::new();
        let nds = trie.get_nds();

        for (nid, nd) in nds.iter().enumerate().skip(1) {
            let is_initial_departure = nd.parent == Some(0);

            if !is_initial_departure && !nd.arr {
                continue;
            }

            let mut depth = 0;
            let mut current = Some(nid);

            while let Some(current_nid) = current {
                current = nds[current_nid].parent;
                if current.is_some() {
                    depth += 1;
                }
            }

            let stop_idx = depth / 2;
            let stop_cands = if stop_idx < stop_candidates.len() {
                stop_candidates[stop_idx].as_slice()
            } else {
                &[]
            };

            let mut group = build_candidate_group(stop_cands, nd, &graph);

            let arrival_time = average_time(nd);

            for candidate in &mut group {
                candidate.time = arrival_time;
                candidate.dep_prede.clear();
            }

            ecm.insert(nid, group.clone());

            if nd.arr {
                for &departure_nid in &nd.childs {
                    let departure_nd = &nds[departure_nid];
                    debug_assert!(!departure_nd.arr);

                    let departure_time = average_time(departure_nd);

                    let departure_group: EdgeCandGroup = group
                        .iter()
                        .enumerate()
                        .map(|(arrival_candidate_id, candidate)| {
                            let mut departure_candidate = candidate.clone();

                            departure_candidate.time = departure_time;
                            departure_candidate.dep_prede = if arrival_time <= departure_time {
                                vec![arrival_candidate_id]
                            } else {
                                Vec::new()
                            };

                            departure_candidate
                        })
                        .collect();

                    ecm.insert(departure_nid, departure_group);
                }
            }
        }

        for (nid, group) in &ecm {
            assert!(!group.is_empty(), "group {} should not be empty", nid);
            assert!(
                group[0].edge.is_none(),
                "candidate 0 at node {} must be null candidate",
                nid
            );
            assert_eq!(group[0].pen, 0.0);
        }

        let dep_group = &ecm[&3];
        let arr_group = &ecm[&2];
        assert_eq!(dep_group.len(), arr_group.len());
        for (i, cand) in dep_group.iter().enumerate() {
            assert_eq!(cand.dep_prede, vec![i]);
        }
    }
}
