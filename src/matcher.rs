use ahash::{AHashMap, AHashSet};
use geo::Point;
use geo::algorithm::HaversineDistance;
use gtfs_structures::RouteType;

use crate::graph::{MODE_BUS, MODE_FERRY, MODE_GONDOLA, MODE_RAIL, MODE_SUBWAY, MODE_TRAM};
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

            let sample_trip_id = &gtfs.patterns.get(pattern).unwrap()[0];
            let trip = gtfs.gtfs.trips.get(sample_trip_id).unwrap();
            let route = gtfs.gtfs.routes.get(&trip.route_id).unwrap();

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

            let mut stop_coords = Vec::new();
            for stop_id in &pattern.stop_ids {
                if let Some(stop) = gtfs.gtfs.stops.get(stop_id) {
                    if let (Some(lat), Some(lon)) = (stop.latitude, stop.longitude) {
                        stop_coords.push(Point::new(lon, lat));
                    }
                }
            }

            if stop_coords.len() < 2 {
                return None;
            }

            let allowed_modes = match pattern.route_type {
                // Trams can sometimes use bus lanes/road, but mostly track. Sticking to TRAM ensures we prefer rails.
                Some(RouteType::Tramway) => MODE_TRAM, 
                Some(RouteType::Subway) => MODE_SUBWAY,
                Some(RouteType::Rail) => MODE_RAIL,
                Some(RouteType::Ferry) => MODE_FERRY,
                Some(RouteType::Gondola)
                | Some(RouteType::Funicular)
                | Some(RouteType::CableCar) => MODE_GONDOLA,
                _ => MODE_BUS,
            };

            let index_to_use = match pattern.route_type {
                Some(RouteType::Tramway) => osm.tram_tree.as_ref().or(osm.bus_tree.as_ref()),
                Some(RouteType::Subway) => osm.metro_tree.as_ref(),
                Some(RouteType::Rail) => osm.rail_tree.as_ref(),
                Some(RouteType::Ferry) => osm.ferry_tree.as_ref(),
                Some(RouteType::Gondola)
                | Some(RouteType::Funicular)
                | Some(RouteType::CableCar) => osm.gondola_tree.as_ref(),
                // Bus, Ferry, etc uses road network
                _ => osm.bus_tree.as_ref(), 
            };

            if index_to_use.is_none() {
                return None;
            }
            let index = index_to_use.unwrap();

            let mut stop_candidates: Vec<Vec<usize>> = Vec::new();
            
            // We fetch more candidates initially, then filter down to a smaller set of diverse candidates.
            const SEARCH_RADIUS_NODES: usize = 100;
            const TARGET_CANDIDATES: usize = 20;
            // For rail, we might need a much larger search radius if the track is far from the stop coordinate (up to ~300m).
            const SEARCH_RADIUS_NODES_RAIL: usize = 4000;

            for point in &stop_coords {
                let search_limit = if pattern.route_type == Some(RouteType::Rail) {
                    SEARCH_RADIUS_NODES_RAIL
                } else {
                    SEARCH_RADIUS_NODES
                };

                let neighbors = index
                    .nearest_neighbor_iter(&[point.x(), point.y()])
                    .take(search_limit);

                let mut candidates: Vec<usize> = Vec::new();
                let mut matched_candidates: Vec<usize> = Vec::new();
                let mut other_candidates: Vec<usize> = Vec::new();

                let mut seen_ways: AHashSet<i64> = AHashSet::new();
                let mut fallback_candidates: Vec<usize> = Vec::new();

                for sn in neighbors {
                    // For rail, the index gives us nearest neighbors, but valid tracks might be a bit further away (up to 200m).
                    // We iterate until we find enough valid ones or exceed max distance.
                    if pattern.route_type == Some(RouteType::Rail) {
                        let node_pl = &osm.graph.node(sn.index).payload;
                        let dist = node_pl
                            .point
                            .haversine_distance(&Point::new(point.x(), point.y()));
                        if dist > 500.0 {
                            break;
                        }
                    }

                    let node_idx = sn.index;
                    let node = osm.graph.node(node_idx);
                    let mut node_matches = false;
                    
                    for &edge_idx in &node.edges {
                        let edge = osm.graph.edge(edge_idx);
                        for line in &edge.payload.lines {
                            let mut line_matches = false;
                            
                            if let (Some(target), Some(line_name)) =
                                (&preferred_match.short_name, Some(&line.short_name))
                            {
                                if target.contains(&line_name.to_lowercase())
                                    || line_name.to_lowercase().contains(target)
                                {
                                    line_matches = true;
                                }
                            }

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

                    let mut node_ways = Vec::new();
                    for &edge_idx in &node.edges {
                        let edge = osm.graph.edge(edge_idx);
                        if edge.payload.osmid != 0 {
                            node_ways.push(edge.payload.osmid);
                        }
                    }

                    let is_new_way = node_ways.iter().any(|w| !seen_ways.contains(w));

                    // We mark ways as seen so we don't spam the same way with candidates, ensuring diversity.
                    if node_matches {
                        matched_candidates.push(node_idx);
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

                if candidates.len() < 5 {
                    let needed = 5 - candidates.len();
                    for &fb in fallback_candidates.iter().take(needed) {
                        candidates.push(fb);
                    }
                }

                stop_candidates.push(candidates);
            }

            if stop_candidates.len() != stop_coords.len()
                || stop_candidates.iter().any(|c| c.is_empty())
            {
                if stop_candidates.len() != stop_coords.len() {
                    return None;
                }
            }

            let mut full_path_geometry = Vec::new();
            let mut relation_found = false;

            // Vote for relations based on ALL candidates.
            // Scoring: Calculate coverage ratio (stops covered / total stops).
            // Map: Relation Index -> (Coverage Score, Candidate Count).
            let mut relation_scores: AHashMap<usize, (f64, usize)> = AHashMap::new();

            for candidates in &stop_candidates {
                let mut seen_for_stop = AHashSet::new();
                for &node_idx in candidates {
                    if let Some(rels) = osm.node_to_relations.get(&node_idx) {
                        for &r_idx in rels {
                            if seen_for_stop.insert(r_idx) {
                                let entry = relation_scores.entry(r_idx).or_insert((0.0, 0));
                                entry.0 += 1.0;
                            }
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

                let get_match_score = |r_idx: usize| -> u8 {
                    let rel = &osm.relations[r_idx];
                    let mut match_score = 0;

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
                let mut current_node_idx_opt = None; 

                if let Some(first_candidates) = stop_candidates.first() {
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
                    continue; 
                }

                let mut possible = true;
                let mut current_node = current_node_idx_opt.unwrap();

                for i in 0..stop_candidates.len() - 1 {
                    let next_candidates = &stop_candidates[i + 1];
                    let next_node_opt = next_candidates.iter().find(|&n| rel.nodes.contains(n));

                    if let Some(&next_node) = next_node_opt {
                        if current_node == next_node {
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
                        // Creates a gap. Relation doesn't fully cover the route.
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
                // Fallback to Backtracking Pathfinding (Viterbi) to handle dead ends.
                // Limit search space for performance: take only top 5 candidates per stop.
                let limited_candidates: Vec<Vec<usize>> = stop_candidates
                    .iter()
                    .map(|c| c.iter().take(5).cloned().collect())
                    .collect();

                if let Some(geometry) = match_sequence_globally_optimal(
                    &limited_candidates,
                    osm,
                    allowed_modes,
                    Some(&preferred_match),
                ) {
                    full_path_geometry = geometry;
                }
            }

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

    // Dynamic Programming / Viterbi Algorithm to minimize total path cost.
    // State: (stop_idx, candidate_idx)
    // MinCost[i][k] = minimum cost to reach candidate k at stop i from start
    // Parent[i][k] = index of candidate at i-1 that leads to MinCost

    let num_stops = stop_candidates.len();
    let mut min_costs: Vec<Vec<Option<(f64, usize)>>> = Vec::with_capacity(num_stops);

    let first_candidates_len = stop_candidates[0].len();
    let mut first_costs = Vec::with_capacity(first_candidates_len);
    for _ in 0..first_candidates_len {
        first_costs.push(Some((0.0, 0))); // Cost 0 to start here. Parent 0 (dummy)
    }
    min_costs.push(first_costs);

    for i in 1..num_stops {
        let prev_candidates = &stop_candidates[i - 1];
        let curr_candidates = &stop_candidates[i];
        let mut curr_costs = vec![None; curr_candidates.len()];

        let mut any_reachable = false;

        for (prev_k, prev_cost_opt) in min_costs[i - 1].iter().enumerate() {
            if let Some((prev_total_cost, _)) = prev_cost_opt {
                let prev_node = prev_candidates[prev_k];

                for (curr_k, &curr_node) in curr_candidates.iter().enumerate() {
                    let cost_inc: f64;

                    if prev_node == curr_node {
                        cost_inc = 0.0;
                    } else {
                        // Note: This can be slow if we have many candidates (N*N pathfinds per stop).
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
            return None;
        }
    }

    let last_stop_idx = num_stops - 1;
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
        // Reconstruct path.
        // We re-run pathfind during backtracking rather than storing geometry for all N*N pairs to save memory.
        let mut full_geometry: Vec<(f64, f64)> = Vec::new();
        let mut segments: Vec<Vec<(f64, f64)>> = Vec::new();

        for i in (1..num_stops).rev() {
            let prev_k = min_costs[i][curr_k].unwrap().1;

            let curr_node = stop_candidates[i][curr_k];
            let prev_node = stop_candidates[i - 1][prev_k];

            let mut segment_geom = Vec::new();

            if curr_node != prev_node {
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
                    return None;
                }
            }
            segments.push(segment_geom);
            curr_k = prev_k;
        }

        let start_node = stop_candidates[0][curr_k];
        let p = osm.graph.node(start_node).payload.point;
        full_geometry.push((p.y(), p.x()));

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
        // Create a simple graph:
        // Track A: 0 -> 1 -> Dead End
        // Track B: 2 -> 3 -> 4 (Success)
        // This tests if the matcher avoids local optimum (closest stops 0,1) for global validity (path exists 2->3->4).

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

        e.geometry = LineString::new(vec![
            Point::new(0.0, 0.0).into(),
            Point::new(1.0, 0.0).into(),
        ]);
        graph.add_edge(n0, n1, e.clone());

        // Edges B
        e.geometry = LineString::new(vec![
            Point::new(0.0, 1.0).into(),
            Point::new(1.0, 1.0).into(),
        ]);
        graph.add_edge(n2, n3, e.clone());

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
            gondola_tree: None,
            relations: Vec::new(),
            node_to_relations: AHashMap::new(),
        };

        // Candidates
        // Stop 1: [0, 2] (0 is closer/first)
        // Stop 2: [1, 3] (1 is closer/first)
        // Stop 3: [4]
        let stop_candidates = vec![vec![n0, n2], vec![n1, n3], vec![n4]];

        let result = match_sequence_globally_optimal(&stop_candidates, &osm, 255, None);

        assert!(result.is_some());
        let points = result.unwrap();
        assert!(points.len() > 1);
    }
}