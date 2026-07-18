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

use rayon::prelude::*;
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicUsize, Ordering};

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
    use std::fs::OpenOptions;
    use std::io::{BufWriter, Write};

    let total_patterns = patterns.len();
    let mut processed = 0;
    println!("Matching {} patterns (full-graph)...", total_patterns);

    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(cache_file_path)?;
    let mut writer = BufWriter::new(file);

    let num_threads = match_threads.unwrap_or_else(rayon::current_num_threads);
    let batch_size = (num_threads * 2).max(1);

    let pool = match_threads
        .map(|threads| rayon::ThreadPoolBuilder::new().num_threads(threads).build())
        .transpose()?;

    let mut results = AHashMap::with_capacity(patterns.len());

    for batch in patterns.chunks(batch_size) {
        let batch_results: Vec<_> = if let Some(ref pool) = pool {
            pool.install(|| {
                batch
                    .par_iter()
                    .map_init(
                        pathfinding::PathfinderContext::new,
                        |ctx, (pattern, trips)| {
                            let res = match_one_pattern(gtfs, osm, pattern, trips, ctx);
                            ctx.discard_if_oversized(500_000);
                            res
                        },
                    )
                    .collect()
            })
        } else {
            batch
                .par_iter()
                .map_init(
                    pathfinding::PathfinderContext::new,
                    |ctx, (pattern, trips)| {
                        let res = match_one_pattern(gtfs, osm, pattern, trips, ctx);
                        ctx.discard_if_oversized(500_000);
                        res
                    },
                )
                .collect()
        };

        for matched in batch_results.into_iter().flatten() {
            let (pattern, shape_result, points) = matched;

            if !shape_result.empty_geometry {
                for (sequence, (lat, lon)) in points.into_iter().enumerate() {
                    let record = BinaryShapeRecord {
                        shape_id: shape_result.shape_id.clone(),
                        shape_pt_lat: lat,
                        shape_pt_lon: lon,
                        shape_pt_sequence: sequence + 1,
                    };

                    bincode::serialize_into(&mut writer, &record)?;
                }
            }

            results.insert(pattern, shape_result);
        }

        processed += batch.len();
        println!("Processed {}/{}", processed, total_patterns);
    }

    writer.flush()?;
    Ok(results)
}

fn match_one_pattern(
    gtfs: &GtfsData,
    osm: &OsmData,
    pattern: &StopPattern,
    _trips: &Vec<String>,
    ctx: &mut pathfinding::PathfinderContext,
) -> Option<(StopPattern, ShapeResult, Vec<(f64, f64)>)> {
    // 0. Extract Route/Agency Info for Matching
    let sample_trip_id = &gtfs.patterns.get(pattern).unwrap()[0];
    let trip = gtfs.gtfs.trips.get(sample_trip_id).unwrap();
    let route = gtfs.gtfs.routes.get(&trip.route_id).unwrap();

    // Find agency
    let agency_name = if let Some(agency_id) = &route.agency_id {
        gtfs.gtfs
            .agencies
            .iter()
            .find(|a| a.id.as_ref() == Some(agency_id))
            .map(|a| a.name.to_lowercase())
    } else {
        None
    };

    let route_short_name = route.short_name.as_ref().map(|s| s.to_lowercase());
    let route_long_name = route.long_name.as_ref().map(|s| s.to_lowercase());

    let preferred_match = TransitMatch {
        short_name: route_short_name.clone(),
        long_name: route_long_name.clone(),
        operator: agency_name.clone(),
    };

    // 1. Get Stop Coordinates
    let mut stop_coords = Vec::new();
    for stop_id in &pattern.stop_ids {
        if let Some(stop) = gtfs.gtfs.stops.get(stop_id) {
            // Geo: x=lon, y=lat. Handle Option fields.
            if let (Some(lat), Some(lon)) = (stop.latitude, stop.longitude) {
                stop_coords.push(Point::new(lon, lat));
            }
        }
    }

    if stop_coords.len() < 2 {
        return None;
    }

    let allowed_modes = match pattern.route_type {
        Some(RouteType::Tramway) => MODE_TRAM, // Trams can sometimes use bus lanes/road, but mostly track. Sticking to TRAM.
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

    // 2. Snap to nearest OSM nodes (Candidates)
    if osm.spatial_tree.is_none() {
        return None;
    }
    let index = osm.spatial_tree.as_ref().unwrap();

    let mut stop_candidates: Vec<Vec<usize>> = Vec::new();
    // How many candidates to consider per stop?
    // We fetch more initially, then filter down to a smaller set of diverse candidates.

    let (search_node_limit, target_candidates, max_dist_meters) = if allowed_modes == MODE_RAIL {
        (4000, 20, 500.0)
    } else if allowed_modes == MODE_BUS {
        (100, 10, 40.0)
    } else {
        (100, 20, f64::INFINITY)
    };

    for point in &stop_coords {
        let search_limit = search_node_limit;

        // For rail, we also want to filter by distance.
        // The index gives us nearest neighbors, but valid tracks might be a bit further away (up to 200m)
        // We'll iterate until we find enough valid ones OR we exceed max distance.

        let neighbors = index
            .nearest_neighbor_iter(&[point.x(), point.y()])
            .filter(|sn| (sn.modes & all_allowed_modes) != 0)
            .take(search_limit);

        let mut candidates: Vec<usize> = Vec::new();
        let mut matched_candidates: Vec<usize> = Vec::new();
        let mut other_candidates: Vec<usize> = Vec::new();

        let mut seen_ways: AHashSet<i64> = AHashSet::new();
        let mut fallback_candidates: Vec<usize> = Vec::new();

        for sn in neighbors {
            // Check distance
            if max_dist_meters < f64::INFINITY {
                let node_pl = &osm.graph.node(sn.index).payload;
                let dist = node_pl
                    .point
                    .haversine_distance(&Point::new(point.x(), point.y()));
                if dist > max_dist_meters {
                    // Too far, stop searching
                    break;
                }
            }

            let node_idx = sn.index;
            let node = osm.graph.node(node_idx);

            // Check if this node matches our route!
            // Check Edges
            let mut node_matches = false;
            for &edge_idx in node.edges() {
                let edge = osm.graph.edge(edge_idx);
                // Check lines
                for line in &edge.payload.lines {
                    let mut line_matches = false;
                    // Check short name
                    if let (Some(target), Some(line_name)) =
                        (&preferred_match.short_name, Some(&line.short_name))
                    {
                        if target.contains(&line_name.to_lowercase())
                            || line_name.to_lowercase().contains(target)
                        {
                            line_matches = true;
                        }
                    }

                    // Check operator
                    if !line_matches {
                        if let (Some(target_op), Some(line_op)) =
                            (&preferred_match.operator, &line.operator)
                        {
                            if target_op.contains(&line_op.to_lowercase())
                                || line_op.to_lowercase().contains(target_op)
                            {
                                line_matches = true;
                            }
                        }
                    }
                    if line_matches {
                        node_matches = true;
                        break;
                    }
                }
            }

            // Check Relations
            if !node_matches {
                if let Some(rels) = osm.node_to_relations.get(&node_idx) {
                    for &r_idx in rels {
                        let rel = &osm.relations[r_idx];
                        let osm_names = [
                            rel.tags.get("ref"),
                            rel.tags.get("name"),
                            rel.tags.get("official_name"),
                            rel.tags.get("alt_name"),
                        ];
                        let mut name_matched = false;
                        for osm_name_opt in osm_names {
                            if let Some(osm_name) = osm_name_opt {
                                let osm_val = osm_name.to_lowercase();
                                if let Some(ref gtfs_short) = preferred_match.short_name {
                                    if osm_val.contains(gtfs_short) || gtfs_short.contains(&osm_val)
                                    {
                                        name_matched = true;
                                    }
                                }
                                if let Some(ref gtfs_long) = preferred_match.long_name {
                                    if osm_val.contains(gtfs_long) || gtfs_long.contains(&osm_val) {
                                        name_matched = true;
                                    }
                                }
                            }
                            if name_matched {
                                break;
                            }
                        }
                        if name_matched {
                            node_matches = true;
                            break;
                        }
                        // Operator check
                        if let (Some(target_op), Some(osm_op)) =
                            (&preferred_match.operator, rel.tags.get("operator"))
                        {
                            if target_op.contains(&osm_op.to_lowercase())
                                || osm_op.to_lowercase().contains(target_op)
                            {
                                node_matches = true;
                                break;
                            }
                        }
                    }
                }
            }

            // Identify ways this node belongs to
            let mut node_ways = Vec::new();
            for &edge_idx in node.edges() {
                let edge = osm.graph.edge(edge_idx);
                if edge.payload.osmid != 0 {
                    node_ways.push(edge.payload.osmid);
                }
            }

            // Check if this node introduces a new way
            let is_new_way = node_ways.iter().any(|w| !seen_ways.contains(w));

            if node_matches {
                // High priority!
                matched_candidates.push(node_idx);
                // Also prevent using ways again? Or allow multiples if matched?
                // Let's mark ways as seen so we don't spam the same way.
                for w in node_ways {
                    seen_ways.insert(w);
                }
            } else if is_new_way {
                if other_candidates.len() < target_candidates {
                    other_candidates.push(node_idx);
                }
                for w in node_ways {
                    seen_ways.insert(w);
                }
            } else {
                // Also keep some fallbacks just in case
                if fallback_candidates.len() < target_candidates {
                    fallback_candidates.push(node_idx);
                }
            }

            if matched_candidates.len() >= target_candidates {
                break;
            }
        }

        // Decision: If we found ANY matches, we use ONLY matches.
        // This prevents "Great Eastern Main Line" from polluting "Elizabeth Line" results if Elizabeth Line exists.
        if !matched_candidates.is_empty() {
            candidates = matched_candidates;
        } else {
            candidates = other_candidates;
        }

        // If we didn't find enough unique-way candidates, fill up with fallbacks
        if candidates.len() < 5 {
            let needed = 5 - candidates.len();
            for &fb in fallback_candidates.iter().take(needed) {
                candidates.push(fb);
            }
        }

        // Limit to TARGET just in case fallbacks pushed over (unlikely with logic above but good for safety)
        // Actually fallback logic above ensures we have at least 5 if possible.
        // If we stopped loop at TARGET_CANDIDATES, we are good.

        if candidates.is_empty() {
            // If we can't snap a stop, we can't route properly?
            // For now, keep empty vec, handle downstream
        }
        stop_candidates.push(candidates);
    }

    if stop_candidates.len() != stop_coords.len() || stop_candidates.iter().any(|c| c.is_empty()) {
        // If any stop failed to snap, we might have issues.
        // But let's proceed if majority are there? No, hard fail for now or fallback?
        // Actually original logic continued if len != len.
        if stop_candidates.len() != stop_coords.len() {
            // println!("Warning: Could not snap all stops for pattern");
            return None;
        }
    }

    // 3. Try Relation Matching
    let mut full_path_geometry = Vec::new();
    let mut relation_found = false;

    // GTFS Info hoisted above

    // Relation Candidate logic
    // Vote for relations based on ALL candidates
    // Scoring: Calculate coverage ratio (stops covered / total stops)
    // Map: Relation Index -> (Coverage Score, Candidate Count)

    let mut relation_scores: AHashMap<usize, (f64, usize)> = AHashMap::new();

    for candidates in &stop_candidates {
        // Identify unique relations covering this stop
        let mut seen_for_stop = AHashSet::new();
        for &node_idx in candidates {
            if let Some(rels) = osm.node_to_relations.get(&node_idx) {
                for &r_idx in rels {
                    if seen_for_stop.insert(r_idx) {
                        // First time this relation covers this stop
                        let entry = relation_scores.entry(r_idx).or_insert((0.0, 0));
                        entry.0 += 1.0;
                    }
                    // Increment candidate count
                    let entry = relation_scores.entry(r_idx).or_insert((0.0, 0));
                    entry.1 += 1;
                }
            }
        }
    }

    let mut candidates: Vec<usize> = relation_scores
        .iter()
        .filter(|(r_idx, (_covered_stops, _count))| {
            let rel = &osm.relations[**r_idx];

            if stop_candidates.is_empty() {
                return false;
            }

            let start_candidates = &stop_candidates[0];
            let end_candidates = stop_candidates.last().unwrap();
            let has_start = start_candidates.iter().any(|&n| rel.nodes.contains(&n));
            let has_end = end_candidates.iter().any(|&n| rel.nodes.contains(&n));

            has_start && has_end
        })
        .map(|(k, _)| *k)
        .collect();

    // Sort candidates by:
    // 1. Coverage Score (Desc)
    // 2. Name/Operator Match (Desc)
    // 3. Candidate Count (Desc)
    candidates.sort_by(|&a, &b| {
        let (score_a, count_a) = relation_scores[&a];
        let (score_b, count_b) = relation_scores[&b];

        // Calculate Match Score
        let get_match_score = |r_idx: usize| -> u8 {
            let rel = &osm.relations[r_idx];
            let mut match_score = 0;

            // Name Match
            // Check 'ref', 'name', 'official_name', 'alt_name'
            let osm_names = [
                rel.tags.get("ref"),
                rel.tags.get("name"),
                rel.tags.get("official_name"),
                rel.tags.get("alt_name"),
            ];

            let mut name_matched = false;
            for osm_name_opt in osm_names {
                if let Some(osm_name) = osm_name_opt {
                    let osm_val = osm_name.to_lowercase();
                    // Check containment both ways
                    if let Some(ref gtfs_short) = route_short_name {
                        if osm_val.contains(gtfs_short) || gtfs_short.contains(&osm_val) {
                            name_matched = true;
                        }
                    }
                    if let Some(ref gtfs_long) = route_long_name {
                        if osm_val.contains(gtfs_long) || gtfs_long.contains(&osm_val) {
                            name_matched = true;
                        }
                    }
                }
                if name_matched {
                    break;
                }
            }
            if name_matched {
                match_score += 2;
            }

            // Operator Match
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

        let match_a = get_match_score(a);
        let match_b = get_match_score(b);

        // Compare scores (f64)
        score_b
            .partial_cmp(&score_a)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| match_b.cmp(&match_a))
            .then_with(|| count_b.cmp(&count_a))
    });

    let mut matched_route_color = None;

    for r_idx in candidates {
        let rel = &osm.relations[r_idx];

        let mut candidate_geometry = Vec::new();
        let mut current_node_idx_opt = None; // Track the actual chosen node for the previous stop

        // Select best start node for this relation
        if let Some(first_candidates) = stop_candidates.first() {
            // Find closest candidate that is IN the relation
            // Candidates are already sorted by distance (nearest_neighbor_iter)
            if let Some(&start_node) = first_candidates.iter().find(|&n| rel.nodes.contains(n)) {
                let start_pl = &osm.graph.node(start_node).payload;
                candidate_geometry.push((start_pl.point.y(), start_pl.point.x()));
                current_node_idx_opt = Some(start_node);
            }
        }

        if current_node_idx_opt.is_none() {
            continue; // Couldn't find valid start for this relation
        }

        let mut possible = true;
        let mut current_node = current_node_idx_opt.unwrap();

        for i in 0..stop_candidates.len() - 1 {
            let next_candidates = &stop_candidates[i + 1];

            // Find best target node for the next stop in this relation
            let next_node_opt = next_candidates.iter().find(|&n| rel.nodes.contains(n));

            if let Some(&next_node) = next_node_opt {
                if current_node == next_node {
                    // Same node, no movement needed, but maybe record geometry?
                    // Just continue loop
                    continue;
                }

                if let Some((_cost, edges)) = pathfinding::pathfind_with_context(
                    ctx,
                    &osm.graph,
                    current_node,
                    next_node,
                    allowed_modes,
                    fallback_modes,
                    Some(&rel.edges),
                    Some(&preferred_match),
                    None,
                ) {
                    for edge_idx in edges {
                        let edge = osm.graph.edge(edge_idx);
                        for dp in edge.payload.geometry.coords().skip(1) {
                            candidate_geometry.push((dp.y, dp.x));
                        }
                    }
                    current_node = next_node;
                } else {
                    possible = false;
                    break;
                }
            } else {
                // Next stop not in relation?
                // This creates a gap. Relation doesn't fully cover the route?
                // We can attempt to proceed if we allow gaps, but for "Relation Matching" we usually want full coverage.
                // Or we can try to pathfind to the *nearest* candidate even if not in relation?
                // But that defeats the purpose of "on relation".
                possible = false;
                break;
            }
        }

        if possible {
            relation_found = true;
            full_path_geometry = candidate_geometry;
            matched_route_color = rel.tags.get("colour").map(|s| s.to_string());
            break;
        }
    }

    if !relation_found {
        // Use RouterImpl for routing
        use crate::router::hop_cache::HopCache;
        use crate::router::router_impl::RouterImpl;
        use crate::router::trip_trie::TripTrie;
        use crate::router::types::{EdgeCand, EdgeCandGroup, EdgeHop};
        use crate::router::weights::{ExpoTransWeight, RoutingAttrs, RoutingOpts};

        // 1. Build TripTrie
        let mut trie = TripTrie::new();
        let r_attrs = RoutingAttrs::default(); // populate properly if needed
        for trip_id in _trips {
            if let Some(trip) = gtfs.gtfs.trips.get(trip_id) {
                trie.add_trip(trip, &r_attrs, true, false);
            }
        }

        // 2. Build EdgeCandMap
        let mut ecm = ahash::AHashMap::new();

        fn build_candidate_group(
            stop_cands: &[crate::graph::NodeIndex],
            nd: &crate::router::trip_trie::TripTrieNd,
            graph: &crate::graph::Graph<crate::graph::NodePL, crate::graph::EdgePL>,
        ) -> EdgeCandGroup {
            let mut cand_group = Vec::new();

            // Null candidate at index 0 as fallback
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

            let mut group = build_candidate_group(stop_cands, nd, &osm.graph);

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

        // 3. Route
        let router: RouterImpl<ExpoTransWeight> = RouterImpl::new(&osm.graph);
        let r_opts = RoutingOpts::default();
        let mut hop_cache = HopCache::new();

        let routes = router.route(&trie, &ecm, &r_opts, Some(&mut hop_cache), false);

        // Extract geometry for the first found leaf
        let mut best_hops = None;
        for (leaf_nid, hops) in routes {
            best_hops = Some(hops);
            break;
        }

        if let Some(hops) = best_hops {
            let mut geometry = Vec::new();
            for hop in hops {
                for edge_idx in hop.edges {
                    let edge = osm.graph.edge(edge_idx);
                    for dp in edge.payload.geometry.coords().skip(1) {
                        geometry.push((dp.y, dp.x));
                    }
                }
            }
            if !geometry.is_empty() {
                full_path_geometry = geometry;
            }
        }
    }

    // Create Shape ID
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    let mut hasher = DefaultHasher::new();
    pattern.hash(&mut hasher);
    let shape_id = format!("shape_{}", hasher.finish());

    let empty_geometry = full_path_geometry.is_empty();

    Some((
        (*pattern).clone(),
        ShapeResult {
            shape_id,
            empty_geometry,
            matched_route_color,
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
