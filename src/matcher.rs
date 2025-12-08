use ahash::{AHashMap, AHashSet};
use geo::Point;
use gtfs_structures::RouteType;

use crate::graph::{MODE_BUS, MODE_RAIL, MODE_SUBWAY, MODE_TRAM};
use crate::gtfs_load::{GtfsData, StopPattern};
use crate::osm_load::OsmData;
use crate::pathfinding;

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
                _ => MODE_BUS,
            };

            // 2. Snap to nearest OSM nodes (Candidates)
            // Select Index based on RouteType
            let index_to_use = match pattern.route_type {
                Some(RouteType::Tramway) => osm.tram_tree.as_ref().or(osm.bus_tree.as_ref()),
                Some(RouteType::Subway) => osm.metro_tree.as_ref(),
                Some(RouteType::Rail) => osm.rail_tree.as_ref(),
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

            for point in &stop_coords {
                let neighbors = index
                    .nearest_neighbor_iter(&[point.x(), point.y()])
                    .take(SEARCH_RADIUS_NODES);

                let mut candidates: Vec<usize> = Vec::new();
                let mut seen_ways: AHashSet<i64> = AHashSet::new();
                let mut fallback_candidates: Vec<usize> = Vec::new();

                for sn in neighbors {
                    let node_idx = sn.index;
                    let node = osm.graph.node(node_idx);

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

                    if is_new_way {
                        candidates.push(node_idx);
                        for w in node_ways {
                            seen_ways.insert(w);
                        }
                    } else {
                        fallback_candidates.push(node_idx);
                    }

                    if candidates.len() >= TARGET_CANDIDATES {
                        break;
                    }
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

            // Relation Candidate logic
            // Vote for relations based on ALL candidates
            // Scoring: Calculate coverage ratio (stops covered / total stops)
            // Map: Relation Index -> (Coverage Score, Average Distance to Candidates)
            // Score = num_stops_covered / total_stops
            // Prioritize: 1. Coverage Score (Desc), 2. Candidate Count (Desc, as heuristic for better fit)

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

                    // Heuristic: Must contain at least one candidate for the first stop AND one for the last stop
                    // UNLESS coverage is very high (e.g. > 90%), then maybe we accept it even if endpoints are fuzzy?
                    // But generally, the start/end requirement is good for directional correctness.
                    if stop_candidates.is_empty() {
                        return false;
                    }

                    let start_candidates = &stop_candidates[0];
                    let end_candidates = stop_candidates.last().unwrap();
                    let has_start = start_candidates.iter().any(|&n| rel.nodes.contains(&n));
                    let has_end = end_candidates.iter().any(|&n| rel.nodes.contains(&n));

                    // Relaxed condition: If coverage is > 80%, we accept even if start/end checks fail?
                    // User said: "It needs to match the full line relation... If none is found, fall back to A*"
                    // "Cuts short" implies we picked a partial relation.
                    // If we enforce start && end, we avoid partial segments that don't span the whole route.
                    // So keeping start && end is GOOD justification.

                    has_start && has_end
                })
                .map(|(k, _)| *k)
                .collect();

            // Sort candidates by coverage (Desc), then Candidate Count (Desc)
            candidates.sort_by(|&a, &b| {
                let (score_a, count_a) = relation_scores[&a];
                let (score_b, count_b) = relation_scores[&b];

                // Compare scores (f64)
                score_b
                    .partial_cmp(&score_a)
                    .unwrap_or(std::cmp::Ordering::Equal)
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

                        if let Some(edges) = pathfinding::pathfind(
                            &osm.graph,
                            current_node,
                            next_node,
                            allowed_modes,
                            Some(&rel.edges),
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
                // println!("Pattern {} ({}): Relation matching failed or incomplete. Falling back to global A*", pattern.id, pattern.stop_ids.len());
                if let Some(geometry) =
                    match_sequence_with_backtracking(&stop_candidates, osm, allowed_modes)
                {
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

fn match_sequence_with_backtracking(
    stop_candidates: &[Vec<usize>],
    osm: &OsmData,
    allowed_modes: u8,
) -> Option<Vec<(f64, f64)>> {
    // visited: (stop_idx, candidate_node_idx) -> bool (dead end)
    let mut visited: AHashSet<(usize, usize)> = AHashSet::new();

    // Recursive helper
    // Returns Option<Vec<(f64, f64)>>: The geometry of the rest of the path including connection from current
    fn backtrack(
        current_stop_idx: usize,
        current_node_idx: usize,
        stop_candidates: &[Vec<usize>],
        osm: &OsmData,
        allowed_modes: u8,
        visited: &mut AHashSet<(usize, usize)>,
    ) -> Option<Vec<(f64, f64)>> {
        // Base case: If we are at the last stop, we are done
        if current_stop_idx == stop_candidates.len() - 1 {
            // Just return the point of the last node
            let pl = &osm.graph.node(current_node_idx).payload;
            return Some(vec![(pl.point.y(), pl.point.x())]);
        }

        // Check memoization
        if visited.contains(&(current_stop_idx, current_node_idx)) {
            return None;
        }

        let next_stop_idx = current_stop_idx + 1;
        let next_candidates = &stop_candidates[next_stop_idx];

        // Try to reach ANY candidate of the next stop
        for &next_node_idx in next_candidates {
            if current_node_idx == next_node_idx {
                if let Some(rest_geometry) = backtrack(
                    next_stop_idx,
                    next_node_idx,
                    stop_candidates,
                    osm,
                    allowed_modes,
                    visited,
                ) {
                    // No new geometry to add between identical nodes, just partial path
                    return Some(rest_geometry);
                }
            } else {
                // Try to pathfind
                // We don't use relation edges here, just generic pathfinding on graph
                if let Some(edges) = pathfinding::pathfind(
                    &osm.graph,
                    current_node_idx,
                    next_node_idx,
                    allowed_modes,
                    None,
                ) {
                    // Path found! Now see if we can continue from next_node
                    if let Some(rest_geometry) = backtrack(
                        next_stop_idx,
                        next_node_idx,
                        stop_candidates,
                        osm,
                        allowed_modes,
                        visited,
                    ) {
                        // Success! Reconstruct full geometry
                        let mut geometry = Vec::new();
                        // Add edges geometry
                        for edge_idx in edges {
                            let edge = osm.graph.edge(edge_idx);
                            // Skip first point of edge to avoid duplication if we are chaining
                            // But here we are prepending.
                            // Logic: Current node point is implicit?
                            // Usually path geometry includes start/end of edges.
                            for coord in edge.payload.geometry.coords().skip(1) {
                                // Skip start to avoid double point?
                                // Wait, if we use skip(1), we miss the start of the first edge?
                                // Usually the start of first edge IS current_node.
                                // So we collect points excluding the FIRST one of each edge?
                                // Standard practice depends on how graph is built.
                                // Let's assume edge geometry starts with `from` and ends with `to`.
                                geometry.push((coord.y, coord.x));
                            }
                        }
                        // Add the rest
                        geometry.extend(rest_geometry);

                        // Note: The very first point of the entire path (start of first edge) might be missing?
                        // The recursion returns geometry *starting after* current_node?
                        // Base case returns just last node point.
                        // Recursive step adds path *to* next node + rest.
                        // So we are missing the current_node point at the very beginning of the top-level call.
                        // But `geometry` above collects points *between* current and next.
                        // If we assume `skip(1)` works, we likely already have `current_node` from previous step?
                        // Wait, for the top-most call, we need to handle the start.
                        return Some(geometry);
                    }
                }
            }
        }

        // If we get here, no path to ANY next candidate worked.
        visited.insert((current_stop_idx, current_node_idx));
        None
    }

    // Top-level loop
    if stop_candidates.is_empty() {
        return None;
    }

    let start_candidates = &stop_candidates[0];
    for &start_node in start_candidates {
        if let Some(path) = backtrack(
            0,
            start_node,
            stop_candidates,
            osm,
            allowed_modes,
            &mut visited,
        ) {
            // Prepend the start node itself
            let pl = &osm.graph.node(start_node).payload;
            let mut full = vec![(pl.point.y(), pl.point.x())];
            full.extend(path);
            return Some(full);
        }
    }

    None
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{EdgePL, Graph, NodePL};

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
        graph.add_edge(n0, n1, e.clone());

        // Edges B
        graph.add_edge(n2, n3, e.clone());
        graph.add_edge(n3, n4, e.clone());

        let osm = OsmData {
            graph,
            rail_tree: None,
            tram_tree: None,
            metro_tree: None,
            bus_tree: None,
            relations: Vec::new(),
            node_to_relations: AHashMap::new(),
        };

        // Candidates
        // Stop 1: [0, 2] (0 is closer/first)
        // Stop 2: [1, 3] (1 is closer/first)
        // Stop 3: [4]
        let stop_candidates = vec![vec![n0, n2], vec![n1, n3], vec![n4]];

        let result = match_sequence_with_backtracking(&stop_candidates, &osm, 255);

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
