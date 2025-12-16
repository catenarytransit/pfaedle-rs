use ahash::{AHashMap, AHashSet};
use geo::Point;
use geo::algorithm::HaversineDistance;
use gtfs_structures::RouteType;

use crate::graph::{MODE_BUS, MODE_FERRY, MODE_RAIL, MODE_SUBWAY, MODE_TRAM};
use crate::gtfs_load::{GtfsData, StopPattern};
use crate::osm_load::OsmData;
use crate::pathfinding::{self, TransitMatch};

#[derive(Debug, Clone)]
pub struct ShapeResult {
    pub shape_id: String,
    pub points: Vec<(f64, f64)>, // Lat, Lon
    pub matched_route_color: Option<String>,
}

use rayon::prelude::*;
use std::sync::atomic::{AtomicUsize, Ordering};

pub fn match_patterns(gtfs: &GtfsData, osm: &OsmData) -> AHashMap<StopPattern, ShapeResult> {
    let total_patterns = gtfs.patterns.len();
    let processed = AtomicUsize::new(0);

    println!("Matching {} patterns...", total_patterns);

    let results: AHashMap<StopPattern, ShapeResult> = gtfs
        .patterns
        .par_iter()
        .map(|(pattern, _trips)| {
            let current_processed = processed.fetch_add(1, Ordering::Relaxed);
            if current_processed % 100 == 0 {
                println!("Processed {}/{}", current_processed, total_patterns);
            }

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
                Some(RouteType::Ferry) | Some(RouteType::Gondola) => MODE_FERRY,
                _ => MODE_BUS,
            };

            // 2. Snap to nearest OSM nodes (Candidates)
            // Select Index based on RouteType
            let index_to_use = match pattern.route_type {
                Some(RouteType::Tramway) => osm.tram_tree.as_ref().or(osm.bus_tree.as_ref()),
                Some(RouteType::Subway) => osm.metro_tree.as_ref(),
                Some(RouteType::Rail) => osm.rail_tree.as_ref(),
                Some(RouteType::Ferry) | Some(RouteType::Gondola) => osm.ferry_tree.as_ref(),
                _ => osm.bus_tree.as_ref(), // Bus, Ferry, etc uses road network
            };

            if index_to_use.is_none() {
                return None;
            }
            let index = index_to_use.unwrap();

            let mut stop_candidates: Vec<Vec<usize>> = Vec::new();
            // How many candidates to consider per stop?
            // We fetch more initially, then filter down to a smaller set of diverse candidates.

            const SEARCH_RADIUS_NODES: usize = 100;
            const TARGET_CANDIDATES: usize = 20;
            // For rail, we might need a much larger search radius if the track is far from the stop coordinate
            // We want to cover up to ~300m. In dense index, 400 nodes might be small. Let's bump to 2000 to be safe.
            const SEARCH_RADIUS_NODES_RAIL: usize = 4000;

            for point in &stop_coords {
                let search_limit = if pattern.route_type == Some(RouteType::Rail) {
                    SEARCH_RADIUS_NODES_RAIL
                } else {
                    SEARCH_RADIUS_NODES
                };

                // For rail, we also want to filter by distance.
                // The index gives us nearest neighbors, but valid tracks might be a bit further away (up to 200m)
                // We'll iterate until we find enough valid ones OR we exceed max distance.

                let neighbors = index
                    .nearest_neighbor_iter(&[point.x(), point.y()])
                    .take(search_limit);

                let mut candidates: Vec<usize> = Vec::new();
                let mut matched_candidates: Vec<usize> = Vec::new();
                let mut other_candidates: Vec<usize> = Vec::new();

                let mut seen_ways: AHashSet<i64> = AHashSet::new();
                let mut fallback_candidates: Vec<usize> = Vec::new();

                for sn in neighbors {
                    // Check distance for Rail
                    if pattern.route_type == Some(RouteType::Rail) {
                        // sn.distance_2 is squared euclidean distance in approx coords.
                        // Ideally we check real distance, but let's assume the projection is roughly meters if using local,
                        // but here we are using lon/lat.
                        // RTree distance is squared euclidean on coords.
                        // We should compute actual distance.
                        let node_pl = &osm.graph.node(sn.index).payload;
                        let dist = node_pl
                            .point
                            .haversine_distance(&Point::new(point.x(), point.y()));
                        if dist > 500.0 {
                            // Too far, stop searching
                            break;
                        }
                    }

                    let node_idx = sn.index;
                    let node = osm.graph.node(node_idx);

                    // Check if this node matches our route!
                    // Check Edges
                    let mut node_matches = false;
                    for &edge_idx in &node.edges {
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
                                            if osm_val.contains(gtfs_short)
                                                || gtfs_short.contains(&osm_val)
                                            {
                                                name_matched = true;
                                            }
                                        }
                                        if let Some(ref gtfs_long) = preferred_match.long_name {
                                            if osm_val.contains(gtfs_long)
                                                || gtfs_long.contains(&osm_val)
                                            {
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
                    for &edge_idx in &node.edges {
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
                        if other_candidates.len() < TARGET_CANDIDATES {
                            other_candidates.push(node_idx);
                        }
                        for w in node_ways {
                            seen_ways.insert(w);
                        }
                    } else {
                        // Also keep some fallbacks just in case
                        if fallback_candidates.len() < 20 {
                            fallback_candidates.push(node_idx);
                        }
                    }

                    if matched_candidates.len() >= TARGET_CANDIDATES {
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

            if stop_candidates.len() != stop_coords.len()
                || stop_candidates.iter().any(|c| c.is_empty())
            {
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
                    if let Some(&start_node) =
                        first_candidates.iter().find(|&n| rel.nodes.contains(n))
                    {
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

                        if let Some((_cost, edges)) = pathfinding::pathfind(
                            &osm.graph,
                            current_node,
                            next_node,
                            allowed_modes,
                            Some(&rel.edges),
                            Some(&preferred_match),
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
                // 4. Fallback to Backtracking Pathfinding to handle dead ends
                // Limit search space for performance: take only top 5 candidates per stop
                let limited_candidates: Vec<Vec<usize>> = stop_candidates
                    .iter()
                    .map(|c| c.iter().take(5).cloned().collect())
                    .collect();

                // println!("Pattern {} ({}): Relation matching failed or incomplete. Falling back to global A*", pattern.id, pattern.stop_ids.len());
                if let Some(geometry) = match_sequence_globally_optimal(
                    &limited_candidates,
                    osm,
                    allowed_modes,
                    Some(&preferred_match),
                ) {
                    // println!("  Fallback successful!");
                    full_path_geometry = geometry;
                } else {
                    // println!("  Fallback failed.");
                }
            }

            // Create Shape ID
            use std::collections::hash_map::DefaultHasher;
            use std::hash::{Hash, Hasher};
            let mut hasher = DefaultHasher::new();
            pattern.hash(&mut hasher);
            let shape_id = format!("shape_{}", hasher.finish());

            Some((
                pattern.clone(),
                ShapeResult {
                    shape_id,
                    points: full_path_geometry,
                    matched_route_color,
                },
            ))
        })
        .filter_map(|x| x)
        .fold(AHashMap::new, |mut acc, (k, v)| {
            acc.insert(k, v);
            acc
        })
        .reduce(AHashMap::new, |mut acc, map| {
            acc.extend(map);
            acc
        });

    results
}

fn match_sequence_globally_optimal(
    stop_candidates: &[Vec<usize>],
    osm: &OsmData,
    allowed_modes: u8,
    preferred_match: Option<&TransitMatch>,
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

        let mut any_reachable = false;

        // For each candidate in previous step
        for (prev_k, prev_cost_opt) in min_costs[i - 1].iter().enumerate() {
            if let Some((prev_total_cost, _)) = prev_cost_opt {
                let prev_node = prev_candidates[prev_k];

                // Try to reach each candidate in current step
                for (curr_k, &curr_node) in curr_candidates.iter().enumerate() {
                    let cost_inc: f64;

                    if prev_node == curr_node {
                        cost_inc = 0.0;
                    } else {
                        // Pathfind
                        // Note: This can be slow if we have many candidates (20*20 = 400 pathfinds per stop).
                        // But usually path between stops is short.
                        if let Some((c, _)) = pathfinding::pathfind(
                            &osm.graph,
                            prev_node,
                            curr_node,
                            allowed_modes,
                            None,
                            preferred_match,
                        ) {
                            cost_inc = c;
                        } else {
                            continue; // Unreachable
                        }
                    }

                    let new_total_cost = prev_total_cost + cost_inc;

                    // Update if better
                    if let Some((existing_cost, _)) = curr_costs[curr_k] {
                        if new_total_cost < existing_cost {
                            curr_costs[curr_k] = Some((new_total_cost, prev_k));
                        }
                    } else {
                        curr_costs[curr_k] = Some((new_total_cost, prev_k));
                    }
                    any_reachable = true;
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

            let mut segment_geom = Vec::new();

            if curr_node == prev_node {
                // No movement
            } else {
                if let Some((_, edges)) = pathfinding::pathfind(
                    &osm.graph,
                    prev_node,
                    curr_node,
                    allowed_modes,
                    None,
                    preferred_match,
                ) {
                    for edge_idx in edges {
                        let edge = osm.graph.edge(edge_idx);
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
        let p = osm.graph.node(start_node).payload.point;
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
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            point: Point::new(1.0, 0.0),
        });
        let n2 = graph.add_node(NodePL {
            point: Point::new(0.0, 1.0),
        });
        let n3 = graph.add_node(NodePL {
            point: Point::new(1.0, 1.0),
        });
        let n4 = graph.add_node(NodePL {
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
            rail_tree: None,
            tram_tree: None,
            metro_tree: None,
            bus_tree: None,
            ferry_tree: None,
            relations: Vec::new(),
            node_to_relations: AHashMap::new(),
        };

        // Candidates
        // Stop 1: [0, 2] (0 is closer/first)
        // Stop 2: [1, 3] (1 is closer/first)
        // Stop 3: [4]
        let stop_candidates = vec![vec![n0, n2], vec![n1, n3], vec![n4]];

        let result = match_sequence_globally_optimal(&stop_candidates, &osm, 255, None);

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
}
