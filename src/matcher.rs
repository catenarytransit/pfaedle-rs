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
    pub matched_route_color: Option<String>,
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

        // Relation Candidate logic
        // Identify potential relations based on snapped nodes
        let mut relation_counts: HashMap<usize, usize> = HashMap::new();
        for &node_idx in &snapped_nodes {
            if let Some(rels) = osm.node_to_relations.get(&node_idx) {
                for &r_idx in rels {
                    *relation_counts.entry(r_idx).or_default() += 1;
                }
            }
        }

        // Candidates must contain start and end nodes? Or just be heavily voted?
        // Let's stick to start and end for strong confidence.
        let mut candidates: Vec<usize> = relation_counts
            .iter()
            .filter(|(r_idx, _count)| {
                let rel = &osm.relations[**r_idx];
                // Check if strict subset of stops?
                // At minimum, let's require seeing at least one node to consider it.
                // Re-using "contains start and end" heuristic is good for end-to-end lines.
                rel.nodes.contains(&snapped_nodes[0])
                    && rel.nodes.contains(snapped_nodes.last().unwrap())
            })
            .map(|(k, _)| *k)
            .collect();

        // Sort candidates by coverage (vote count) descending?
        candidates.sort_by_key(|&idx| std::cmp::Reverse(relation_counts[&idx]));

        let mut matched_route_color = None;

        for r_idx in candidates {
            let rel = &osm.relations[r_idx];

            // Attempt to build path using ONLY relation edges (and nodes) via A*
            // We go from stop to stop.
            let mut candidate_geometry = Vec::new();
            let start_node = snapped_nodes[0];
            let start_pl = &osm.graph.node(start_node).payload;
            candidate_geometry.push((start_pl.point.y(), start_pl.point.x()));

            let mut possible = true;

            for i in 0..snapped_nodes.len() - 1 {
                let u = snapped_nodes[i];
                let v = snapped_nodes[i + 1];

                if u == v {
                    continue;
                }

                if let Some(edges) =
                    pathfinding::pathfind(&osm.graph, u, v, allowed_modes, Some(&rel.edges))
                {
                    for edge_idx in edges {
                        let edge = osm.graph.edge(edge_idx);
                        for dp in edge.payload.geometry.coords().skip(1) {
                            candidate_geometry.push((dp.y, dp.x));
                        }
                    }
                } else {
                    possible = false;
                    break;
                }
            }

            if possible {
                relation_found = true;
                full_path_geometry = candidate_geometry;
                matched_route_color = rel.tags.get("colour").map(|s| s.to_string());
                // println!("Matched relation {} for pattern.", r_idx);
                break;
            }
        }

        if !relation_found {
            // 4. Fallback to Unrestricted A* Pathfinding
            let first_node_pl = &osm.graph.node(snapped_nodes[0]).payload;
            full_path_geometry.push((first_node_pl.point.y(), first_node_pl.point.x()));

            for i in 0..snapped_nodes.len() - 1 {
                let start = snapped_nodes[i];
                let end = snapped_nodes[i + 1];

                if start == end {
                    continue;
                }

                if let Some(edges) =
                    pathfinding::pathfind(&osm.graph, start, end, allowed_modes, None)
                {
                    for edge_idx in edges {
                        let edge = osm.graph.edge(edge_idx);
                        for coord in edge.payload.geometry.coords().skip(1) {
                            full_path_geometry.push((coord.y, coord.x));
                        }
                    }
                } else {
                    // A* failed completely
                    // Draw straight line to next stop
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

        // Extract color if available from the best candidate relation
        // (Handled inside loop via matched_route_color)

        results.insert(
            pattern.clone(),
            ShapeResult {
                shape_id,
                points: full_path_geometry,
                matched_route_color,
            },
        );

        // Refactoring applied below to the whole match_patterns function involves significant change.
        // Instead, I'll update the loop to capture the color.

        // Wait, I can't easily reference the loop variable 'rel' after the loop.
        // I need to update the ReplacementContent to include the loop logic change or do it in two steps.
        // Since I have valid context, I will replace the struct definition AND the function content at once?
        // No, the file is large.
        // I will use `replace_file_content` for the struct definition first.
    }

    results
}
