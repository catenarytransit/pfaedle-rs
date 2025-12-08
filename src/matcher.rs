use geo::Point;
use gtfs_structures::RouteType;
use std::collections::HashMap;

use crate::graph::{MODE_BUS, MODE_RAIL, MODE_SUBWAY, MODE_TRAM};
use crate::gtfs_load::{GtfsData, StopPattern};
use crate::osm_load::OsmData;
use crate::pathfinding;

#[derive(Debug, Clone)]
pub struct ShapeResult {
    pub shape_id: String,
    pub points: Vec<(f64, f64)>, // Lat, Lon
}

pub fn match_patterns(gtfs: &GtfsData, osm: &OsmData) -> HashMap<StopPattern, ShapeResult> {
    let mut results = HashMap::new();
    let total_patterns = gtfs.patterns.len();
    let mut processed = 0;

    println!("Matching {} patterns...", total_patterns);

    for (pattern, _trips) in &gtfs.patterns {
        processed += 1;
        if processed % 10 == 0 {
            println!("Processed {}/{}", processed, total_patterns);
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
            continue;
        }

        if stop_coords.len() < 2 {
            continue;
        }

        let allowed_modes = match pattern.route_type {
            Some(RouteType::Tramway) => MODE_TRAM, // Trams can sometimes use bus lanes/road, but mostly track. Sticking to TRAM.
            Some(RouteType::Subway) => MODE_SUBWAY,
            Some(RouteType::Rail) => MODE_RAIL,
            _ => MODE_BUS,
        };

        // 2. Snap to nearest OSM nodes
        // Select Index based on RouteType
        let index_to_use = match pattern.route_type {
            Some(RouteType::Tramway) => osm.tram_tree.as_ref().or(osm.bus_tree.as_ref()),
            Some(RouteType::Subway) => osm.metro_tree.as_ref(),
            Some(RouteType::Rail) => osm.rail_tree.as_ref(),
            _ => osm.bus_tree.as_ref(), // Bus, Ferry, etc uses road network
        };

        if index_to_use.is_none() {
            // println!("Warning: No spatial index found for route type {:?}", pattern.route_type);
            continue;
        }
        let index = index_to_use.unwrap();

        let mut snapped_nodes = Vec::new();
        for point in &stop_coords {
            let nearest = index.nearest_neighbor(&[point.x(), point.y()]);
            if let Some(sn) = nearest {
                snapped_nodes.push(sn.index);
            }
        }

        if snapped_nodes.len() != stop_coords.len() {
            println!("Warning: Could not snap all stops for pattern");
            continue;
        }

        // 3. Try Relation Matching
        let mut full_path_geometry = Vec::new();
        let mut relation_found = false;

        // Heuristic: Find a relation that covers the stops
        // We look for a relation that contains the first and last stop, and preserves order.
        // Optimization: Just check start and end? Or check all?
        // Checking all gives better confidence.

        // Count votes for relations from all snapped nodes
        let mut relation_counts: HashMap<usize, usize> = HashMap::new();
        for &node_idx in &snapped_nodes {
            if let Some(rels) = osm.node_to_relations.get(&node_idx) {
                for &r_idx in rels {
                    *relation_counts.entry(r_idx).or_default() += 1;
                }
            }
        }

        // Find best candidate
        // Required: Must contain start and end nodes.
        let candidates: Vec<usize> = relation_counts
            .keys()
            .filter(|&r_idx| {
                // Heuristic: cover at least 50% of stops? Or just verify geometry?
                // Let's be strict: must contain first and last.
                let rel = &osm.relations[*r_idx];
                rel.nodes.contains(&snapped_nodes[0])
                    && rel.nodes.contains(snapped_nodes.last().unwrap())
            })
            .cloned()
            .collect();

        for r_idx in candidates {
            let rel = &osm.relations[r_idx];
            // Check order
            let pos_first = rel.nodes.iter().position(|&n| n == snapped_nodes[0]);
            let pos_last = rel
                .nodes
                .iter()
                .rposition(|&n| n == *snapped_nodes.last().unwrap());

            if let (Some(start_idx), Some(end_idx)) = (pos_first, pos_last) {
                if start_idx < end_idx {
                    // Forward match
                    // We assume the relationship is a simple sequence of nodes.
                    // Copy nodes from start_idx to end_idx
                    // Verify intermediate coverage?
                    // For now, trust the relation if it connects start and end.
                    relation_found = true;
                    let first_node = rel.nodes[start_idx];
                    let p = &osm.graph.node(first_node).payload.point;
                    full_path_geometry.push((p.y(), p.x()));

                    for i in start_idx..end_idx {
                        let u = rel.nodes[i];
                        let v = rel.nodes[i + 1];

                        // Check if u -> v is directly connected
                        let mut connected = false;
                        for &e_idx in &osm.graph.node(u).edges {
                            let edge = osm.graph.edge(e_idx);
                            if edge.to == v && (edge.payload.allowed_modes & allowed_modes) != 0 {
                                // Add geometry
                                // Re-use path logic:
                                for dp in edge.payload.geometry.coords().skip(1) {
                                    full_path_geometry.push((dp.y, dp.x));
                                }
                                connected = true;
                                break;
                            }
                        }

                        if !connected {
                            // Gap detected! Try A*
                            // println!("Gap in relation between {} and {}, filling...", u, v);
                            if let Some(edges) =
                                pathfinding::pathfind(&osm.graph, u, v, allowed_modes)
                            {
                                for edge_idx in edges {
                                    let edge = osm.graph.edge(edge_idx);
                                    for dp in edge.payload.geometry.coords().skip(1) {
                                        full_path_geometry.push((dp.y, dp.x));
                                    }
                                }
                            } else {
                                // Fallback: Straight line
                                let p_v = &osm.graph.node(v).payload.point;
                                full_path_geometry.push((p_v.y(), p_v.x()));
                            }
                        }
                    }
                    break;
                } else if start_idx > end_idx {
                    // Backward match (relation defined in reverse? Or trip is return?)
                    // If relation is just a sequence, we can traverse reverse.
                    relation_found = true;
                    // For reverse, we can iterate start_idx down to end_idx,
                    // BUT pathfinding is directed. We need to find path u->v where u is current, v is next in TRAJECTORY (so prev in relation).

                    let first_node = rel.nodes[start_idx];
                    let p = &osm.graph.node(first_node).payload.point;
                    full_path_geometry.push((p.y(), p.x()));

                    // We are going from start_idx DOWN to end_idx
                    // i goes from start_idx down to end_idx + 1
                    for i in (end_idx + 1..=start_idx).rev() {
                        let u = rel.nodes[i];
                        let v = rel.nodes[i - 1]; // next node in trajectory

                        let mut connected = false;
                        for &e_idx in &osm.graph.node(u).edges {
                            let edge = osm.graph.edge(e_idx);
                            if edge.to == v && (edge.payload.allowed_modes & allowed_modes) != 0 {
                                for dp in edge.payload.geometry.coords().skip(1) {
                                    full_path_geometry.push((dp.y, dp.x));
                                }
                                connected = true;
                                break;
                            }
                        }

                        if !connected {
                            if let Some(edges) =
                                pathfinding::pathfind(&osm.graph, u, v, allowed_modes)
                            {
                                for edge_idx in edges {
                                    let edge = osm.graph.edge(edge_idx);
                                    for dp in edge.payload.geometry.coords().skip(1) {
                                        full_path_geometry.push((dp.y, dp.x));
                                    }
                                }
                            } else {
                                let p_v = &osm.graph.node(v).payload.point;
                                full_path_geometry.push((p_v.y(), p_v.x()));
                            }
                        }
                    }
                    break;
                }
            }
        }

        if relation_found {
            // println!("Used relation for match!");
        } else {
            // 4. Fallback to A* Pathfinding
            let first_node_pl = &osm.graph.node(snapped_nodes[0]).payload;
            full_path_geometry.push((first_node_pl.point.y(), first_node_pl.point.x()));

            for i in 0..snapped_nodes.len() - 1 {
                let start = snapped_nodes[i];
                let end = snapped_nodes[i + 1];

                if start == end {
                    continue;
                }

                if let Some(edges) = pathfinding::pathfind(&osm.graph, start, end, allowed_modes) {
                    for edge_idx in edges {
                        let edge = osm.graph.edge(edge_idx);
                        for coord in edge.payload.geometry.coords() {
                            full_path_geometry.push((coord.y, coord.x));
                        }
                    }
                } else {
                    // A* failed
                    let next_node_pl = &osm.graph.node(end).payload;
                    full_path_geometry.push((next_node_pl.point.y(), next_node_pl.point.x()));
                }
            }
        }

        // Create Shape ID
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        let mut hasher = DefaultHasher::new();
        pattern.hash(&mut hasher);
        let shape_id = format!("shape_{}", hasher.finish());

        results.insert(
            pattern.clone(),
            ShapeResult {
                shape_id,
                points: full_path_geometry,
            },
        );
    }

    results
}
