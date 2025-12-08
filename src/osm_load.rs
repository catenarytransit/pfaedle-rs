use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use geo::{LineString, Point};
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader, Tags};
use rstar::RTree;

use std::path::Path;

use crate::graph::{
    EdgeIndex, EdgePL, Graph, MODE_BUS, MODE_RAIL, MODE_SUBWAY, MODE_TRAM, NodeIndex, NodePL,
    TransitInfo,
};
use gtfs_structures::RouteType;

// RStar compatible struct for spatial indexing
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpatialNode {
    pub index: NodeIndex,
    pub point: [f64; 2], // [x, y] = [lon, lat]
}

impl rstar::PointDistance for SpatialNode {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.point[0] - point[0];
        let dy = self.point[1] - point[1];
        dx * dx + dy * dy
    }
}

impl rstar::RTreeObject for SpatialNode {
    type Envelope = rstar::AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_point(self.point)
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct OsmRelation {
    pub id: i64,
    pub tags: Tags,
    pub nodes: Vec<NodeIndex>,      // All nodes in the relation
    pub edges: AHashSet<EdgeIndex>, // All edges in the relation
}

pub struct OsmData {
    pub graph: Graph<NodePL, EdgePL>,
    pub rail_tree: Option<RTree<SpatialNode>>,
    pub tram_tree: Option<RTree<SpatialNode>>,
    pub metro_tree: Option<RTree<SpatialNode>>,
    pub bus_tree: Option<RTree<SpatialNode>>,
    pub relations: Vec<OsmRelation>,
    pub node_to_relations: AHashMap<NodeIndex, Vec<usize>>,
}

pub struct OsmBuilder;

impl OsmBuilder {
    pub fn read(
        path: &Path,
        used_route_types: &AHashSet<RouteType>,
        bbox: Option<(f64, f64, f64, f64)>, // min_lon, min_lat, max_lon, max_lat
    ) -> Result<OsmData> {
        println!("Reading OSM file {:?} in multiple passes...", path);

        // --- Data Structures to persist across passes ---

        // Relation metadata found in Pass 1
        struct PreRelation {
            id: i64,
            tags: Tags,
            members: Vec<osmpbfreader::Ref>,
        }
        let mut pre_relations: Vec<PreRelation> = Vec::new();

        // Set of Way IDs that are members of any interesting relation.
        let mut ways_in_relations: AHashSet<i64> = AHashSet::new();

        // Set of Node IDs that are required for the graph.
        let mut needed_nodes: AHashSet<i64> = AHashSet::new();

        // ------------------------------------------------
        // PASS 1: Relations
        // Goal: Identify interesting relations (Routes), store them, and identify which Ways they need.
        // ------------------------------------------------
        println!("Pass 1/4: Scanning relations...");
        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 1")?;
                if let OsmObj::Relation(r) = obj {
                    // Check if relation is interesting
                    let is_route =
                        r.tags.contains_key("route") || r.tags.contains_key("public_transport");
                    if is_route {
                        // Store relevant info
                        for member in &r.refs {
                            if let OsmId::Way(wid) = member.member {
                                ways_in_relations.insert(wid.0);
                            } else if let OsmId::Node(nid) = member.member {
                                needed_nodes.insert(nid.0); // Direct node members
                            } else if let OsmId::Relation(_rid) = member.member {
                                // Relation member - we will handle flattening later,
                                // but we don't need to add ID to 'needed' sets here,
                                // because we already iterate ALL relations.
                                // However, we must ensure we KEEP this member in the PreRelation.
                            }
                        }

                        pre_relations.push(PreRelation {
                            id: r.id.0,
                            tags: r.tags,
                            members: r.refs,
                        });
                    }
                }
            }
        }
        println!(
            "  Found {} relevant relations. Need {} ways.",
            pre_relations.len(),
            ways_in_relations.len()
        );

        // ------------------------------------------------
        // PASS 2: Ways (Discovery)
        // Goal: For every 'infrastructure' way OR 'relation-member' way, mark its nodes as needed.
        // ------------------------------------------------
        println!("Pass 2/4: Scanning ways to identify needed nodes...");
        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 2")?;
                if let OsmObj::Way(w) = obj {
                    let wid = w.id.0;
                    let is_infra = Self::is_infrastructure(&w);
                    let is_platform = Self::is_platform(&w);
                    // Pass 2: We only care about relation members if they are valid geometry (not platforms)
                    let is_rel_member = ways_in_relations.contains(&wid) && !is_platform;

                    if is_infra || is_rel_member {
                        if !Self::is_valid_way(&w) {
                            continue;
                        }

                        // Mark all nodes as needed
                        for nid in &w.nodes {
                            needed_nodes.insert(nid.0);
                        }
                    }
                }
            }
        }
        println!(
            "  Identified {} unique nodes needed for the graph.",
            needed_nodes.len()
        );

        // ------------------------------------------------
        // PASS 3: Nodes
        // Goal: Load only the 'needed_nodes' into the Graph and build the ID map.
        // ------------------------------------------------
        println!("Pass 3/4: Loading nodes...");
        let mut graph = Graph::new();
        // Map from OSM Node ID -> Graph NodeIndex
        let mut osm_node_to_graph_idx: AHashMap<i64, NodeIndex> = AHashMap::new();

        let mut rail_node_indices: AHashSet<NodeIndex> = AHashSet::new();
        let mut tram_node_indices: AHashSet<NodeIndex> = AHashSet::new();
        let mut metro_node_indices: AHashSet<NodeIndex> = AHashSet::new();
        let mut bus_node_indices: AHashSet<NodeIndex> = AHashSet::new();
        let mut stop_node_indices: AHashSet<NodeIndex> = AHashSet::new();

        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 3")?;
                if let OsmObj::Node(n) = obj {
                    let nid = n.id.0;
                    if needed_nodes.contains(&nid) {
                        let lat = n.lat();
                        let lon = n.lon();

                        // Filter by Bounding Box if present
                        if let Some((min_lon, min_lat, max_lon, max_lat)) = bbox {
                            if lon < min_lon || lon > max_lon || lat < min_lat || lat > max_lat {
                                continue;
                            }
                        }

                        let idx = graph.add_node(NodePL {
                            point: Point::new(lon, lat),
                        });
                        osm_node_to_graph_idx.insert(nid, idx);

                        // Identify stop/platform nodes to filter from relations later
                        let is_stop = n
                            .tags
                            .get("railway")
                            .map_or(false, |s| s == "stop" || s == "platform_edge")
                            || n.tags.get("public_transport").map_or(false, |s| {
                                s == "stop_position" || s == "platform" || s == "station"
                            });

                        if is_stop {
                            stop_node_indices.insert(idx);
                        }
                    }
                }
            }
        }
        // clear needed_nodes to free memory
        needed_nodes.clear();
        needed_nodes.shrink_to_fit();

        println!("  Graph loaded with {} nodes.", graph.nodes.len());

        // ------------------------------------------------
        // PASS 4: Edges & Finalizing
        // Goal: Re-scan Ways. Build edges for infrastructure ways.
        //       Cache node-lists for relation ways to rebuild relation geometries.
        // ------------------------------------------------
        println!("Pass 4/4: Building edges and relations...");

        // Cache for ways used in relations: WayID -> Vec<GraphNodeIndex>
        let mut way_id_to_node_indices: AHashMap<i64, Vec<NodeIndex>> = AHashMap::new();
        let mut way_to_edge_indices: AHashMap<i64, Vec<EdgeIndex>> = AHashMap::new();

        // 4a. Build Way -> TransitInfo lookup
        let mut way_transit_info: AHashMap<i64, Vec<TransitInfo>> = AHashMap::new();

        let get_transit_info = |r: &PreRelation| -> Option<TransitInfo> {
            let short_name = r
                .tags
                .get("ref")
                .or_else(|| r.tags.get("name"))
                .map(|s| s.to_string())
                .unwrap_or_default();
            let from_str = r
                .tags
                .get("from")
                .map(|s| s.to_string())
                .unwrap_or_default();
            let to_str = r.tags.get("to").map(|s| s.to_string()).unwrap_or_default();

            if !short_name.is_empty() || !from_str.is_empty() {
                Some(TransitInfo {
                    short_name,
                    from_str,
                    to_str,
                })
            } else {
                None
            }
        };

        for r in &pre_relations {
            if let Some(info) = get_transit_info(r) {
                for member in &r.members {
                    if let OsmId::Way(wid) = member.member {
                        way_transit_info
                            .entry(wid.0)
                            .or_default()
                            .push(info.clone());
                    }
                }
            }
        }
        for infos in way_transit_info.values_mut() {
            infos.sort();
            infos.dedup();
        }

        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 4")?;
                if let OsmObj::Way(w) = obj {
                    let wid = w.id.0;
                    let is_platform = Self::is_platform(&w);
                    // Filter platforms from relations here too
                    let is_rel_member = ways_in_relations.contains(&wid) && !is_platform;
                    let is_infra = Self::is_infrastructure(&w);

                    if !is_infra && !is_rel_member {
                        continue;
                    }
                    if !Self::is_valid_way(&w) {
                        continue;
                    }

                    // Resolve Nodes
                    let mut way_indices = Vec::with_capacity(w.nodes.len());
                    for nid in &w.nodes {
                        if let Some(&idx) = osm_node_to_graph_idx.get(&nid.0) {
                            way_indices.push(idx);
                        }
                    }

                    // Helper to track node types
                    let (is_rail, is_tram, is_metro, is_bus) = Self::classify_way(&w);

                    if !way_indices.is_empty() {
                        // categorize nodes for indices
                        for &idx in &way_indices {
                            if is_rail {
                                rail_node_indices.insert(idx);
                            }
                            if is_tram {
                                tram_node_indices.insert(idx);
                            }
                            if is_metro {
                                metro_node_indices.insert(idx);
                            }
                            if is_bus {
                                bus_node_indices.insert(idx);
                            }
                        }
                    }

                    // Store for Relation Reconstruction if needed
                    if is_rel_member && !way_indices.is_empty() {
                        way_id_to_node_indices.insert(wid, way_indices.clone());
                    }

                    // Build Edges
                    // Only if infrastructure
                    if is_infra && way_indices.len() > 1 {
                        let transit_lines = way_transit_info.get(&wid).cloned().unwrap_or_default();
                        let mut created_edges = Vec::new();

                        for i in 0..way_indices.len() - 1 {
                            let u = way_indices[i];
                            let v = way_indices[i + 1];

                            let p1 = graph.nodes[u].payload.point;
                            let p2 = graph.nodes[v].payload.point;
                            let geom = LineString::new(vec![p1.into(), p2.into()]);

                            let mut edge_pl = EdgePL::new();
                            edge_pl.geometry = geom;
                            edge_pl.lines = transit_lines.clone();
                            edge_pl.level = Self::parse_level(&w.tags);
                            edge_pl.oneway = Self::parse_oneway(&w.tags);

                            let mut modes = 0;
                            if is_rail {
                                modes |= MODE_RAIL;
                            }
                            if is_tram {
                                modes |= MODE_TRAM;
                            }
                            if is_metro {
                                modes |= MODE_SUBWAY;
                            }
                            if is_bus {
                                modes |= MODE_BUS;
                            }
                            edge_pl.allowed_modes = modes;
                            edge_pl.osmid = wid;

                            edge_pl.cost = Self::calculate_cost(&w.tags, edge_pl.length());

                            let idx = graph.add_edge(u, v, edge_pl);
                            created_edges.push(idx);
                        }
                        way_to_edge_indices.insert(wid, created_edges);
                    }
                }
            }
        }

        Self::post_process(&mut graph);

        // Build Output Relations
        let mut final_node_to_rels: AHashMap<NodeIndex, Vec<usize>> = AHashMap::new();
        let mut relations_list: Vec<OsmRelation> = Vec::new();

        let pre_rel_map: AHashMap<i64, &PreRelation> =
            pre_relations.iter().map(|r| (r.id, r)).collect();

        for r_pre in &pre_relations {
            if r_pre.tags.get("type").map(|s| s.as_str()) != Some("route") {
                continue;
            }

            // 5. Recursive Flattening of Relations
            // Map created outside loop.

            // Helper for recursion with cycle detection
            fn flatten_relation(
                r_id: i64,
                pre_rel_map: &AHashMap<i64, &PreRelation>,
                osm_node_to_graph_idx: &AHashMap<i64, NodeIndex>,
                way_id_to_node_indices: &AHashMap<i64, Vec<NodeIndex>>,
                way_to_edge_indices: &AHashMap<i64, Vec<EdgeIndex>>,
                stop_node_indices: &AHashSet<NodeIndex>,
                visited: &mut AHashSet<i64>,
                out_nodes: &mut Vec<NodeIndex>,
                out_edges: &mut AHashSet<EdgeIndex>,
                out_final_node_to_rels: &mut AHashMap<NodeIndex, Vec<usize>>,
                current_rel_idx: usize,
            ) {
                if !visited.insert(r_id) {
                    return; // Cycle detected or already processed
                }

                if let Some(r_pre) = pre_rel_map.get(&r_id) {
                    for member in &r_pre.members {
                        match member.member {
                            OsmId::Node(nid) => {
                                if let Some(&idx) = osm_node_to_graph_idx.get(&nid.0) {
                                    if !stop_node_indices.contains(&idx) {
                                        out_nodes.push(idx);
                                        out_final_node_to_rels
                                            .entry(idx)
                                            .or_default()
                                            .push(current_rel_idx);
                                    }
                                }
                            }
                            OsmId::Way(wid) => {
                                if let Some(nodes) = way_id_to_node_indices.get(&wid.0) {
                                    for &idx in nodes {
                                        out_nodes.push(idx);
                                        out_final_node_to_rels
                                            .entry(idx)
                                            .or_default()
                                            .push(current_rel_idx);
                                    }
                                }
                                if let Some(edges) = way_to_edge_indices.get(&wid.0) {
                                    for &e in edges {
                                        out_edges.insert(e);
                                    }
                                }
                            }
                            OsmId::Relation(sub_rid) => {
                                flatten_relation(
                                    sub_rid.0,
                                    pre_rel_map,
                                    osm_node_to_graph_idx,
                                    way_id_to_node_indices,
                                    way_to_edge_indices,
                                    stop_node_indices,
                                    visited,
                                    out_nodes,
                                    out_edges,
                                    out_final_node_to_rels,
                                    current_rel_idx,
                                );
                            }
                        }
                    }
                }
            }

            let rel_idx = relations_list.len();
            let mut rel_nodes = Vec::new();
            let mut rel_edges = AHashSet::new();
            let mut visited_rels = AHashSet::new();

            flatten_relation(
                r_pre.id,
                &pre_rel_map,
                &osm_node_to_graph_idx,
                &way_id_to_node_indices,
                &way_to_edge_indices,
                &stop_node_indices,
                &mut visited_rels,
                &mut rel_nodes,
                &mut rel_edges,
                &mut final_node_to_rels,
                rel_idx,
            );

            relations_list.push(OsmRelation {
                id: r_pre.id,
                tags: r_pre.tags.clone(),
                nodes: rel_nodes,
                edges: rel_edges,
            });
        }

        let build_tree = |indices: AHashSet<NodeIndex>, name: &str| -> Option<RTree<SpatialNode>> {
            // Filter out stop nodes
            let indices: Vec<NodeIndex> = indices.difference(&stop_node_indices).cloned().collect();

            if indices.is_empty() {
                return None;
            }
            println!("Building {} index with {} nodes...", name, indices.len());
            let nodes: Vec<SpatialNode> = indices
                .into_iter()
                .map(|idx| {
                    let p = graph.nodes[idx].payload.point;
                    SpatialNode {
                        index: idx,
                        point: [p.x(), p.y()],
                    }
                })
                .collect();
            Some(RTree::bulk_load(nodes))
        };

        let needs_rail = used_route_types.contains(&RouteType::Rail);
        let needs_tram = used_route_types.contains(&RouteType::Tramway);
        let needs_metro = used_route_types.contains(&RouteType::Subway);
        let needs_bus = used_route_types.iter().any(|r| {
            *r == RouteType::Bus
                || (*r != RouteType::Rail && *r != RouteType::Tramway && *r != RouteType::Subway)
        });

        let rail_tree = if needs_rail {
            build_tree(rail_node_indices, "Rail")
        } else {
            None
        };
        let tram_tree = if needs_tram {
            build_tree(tram_node_indices, "Tram")
        } else {
            None
        };
        let metro_tree = if needs_metro {
            build_tree(metro_node_indices, "Metro")
        } else {
            None
        };
        let bus_tree = if needs_bus {
            build_tree(bus_node_indices, "Bus")
        } else {
            None
        };

        println!(
            "Graph built: {} nodes, {} edges. Relations: {}",
            graph.nodes.len(),
            graph.edges.len(),
            relations_list.len()
        );

        Ok(OsmData {
            graph,
            rail_tree,
            tram_tree,
            metro_tree,
            bus_tree,
            relations: relations_list,
            node_to_relations: final_node_to_rels,
        })
    }

    fn open_pbf(path: &Path) -> Result<OsmPbfReader<std::fs::File>> {
        let f = std::fs::File::open(path).with_context(|| format!("Failed to open {:?}", path))?;
        Ok(OsmPbfReader::new(f))
    }

    fn post_process(graph: &mut Graph<NodePL, EdgePL>) {
        // writeODirEdgs: Add reverse edges
        let existing_edges: AHashSet<(NodeIndex, NodeIndex)> =
            graph.edges.iter().map(|e| (e.from, e.to)).collect();

        let mut edges_to_add = Vec::new();
        for edge in &graph.edges {
            let u = edge.from;
            let v = edge.to;
            if !existing_edges.contains(&(v, u)) {
                let mut rev_pl = edge.payload.rev_copy();
                // Penalize going backwards a bit (10% penalty)
                rev_pl.cost = (rev_pl.cost as f64 * 1.1) as u32;
                edges_to_add.push((v, u, rev_pl));
            }
        }

        for (from, to, pl) in edges_to_add {
            graph.add_edge(from, to, pl);
        }

        // writeOneWayPens: Penalize forbidden directions
        for edge in &mut graph.edges {
            if edge.payload.oneway == 2 {
                // Severe penalty for wrong direction of one-way
                edge.payload.cost = edge.payload.cost.saturating_mul(100);
            }
        }
    }

    fn has_tag(tags: &Tags, key: &str, val: &str) -> bool {
        tags.get(key).map(|s| s.as_str()) == Some(val)
    }

    fn is_valid_way(w: &osmpbfreader::Way) -> bool {
        w.nodes.len() > 1
    }

    fn is_infrastructure(w: &osmpbfreader::Way) -> bool {
        if Self::is_platform(w) {
            return false;
        }
        // Filter out industrial usage
        if w.tags.get("usage").map_or(false, |u| u == "industrial") {
            return false;
        }
        w.tags.contains_key("railway") || w.tags.contains_key("highway")
    }

    fn is_platform(w: &osmpbfreader::Way) -> bool {
        if let Some(r) = w.tags.get("railway") {
            if r == "platform" || r == "stop" || r == "platform_edge" {
                return true;
            }
        }
        if let Some(pt) = w.tags.get("public_transport") {
            if pt == "platform" || pt == "stop_position" || pt == "station" {
                return true;
            }
        }
        false
    }

    fn classify_way(w: &osmpbfreader::Way) -> (bool, bool, bool, bool) {
        let railway = w.tags.get("railway").map(|s| s.as_str());
        let highway = w.tags.get("highway").map(|s| s.as_str());

        let is_rail = railway.map_or(false, |r| {
            r == "rail" || r == "light_rail" || r == "narrow_gauge"
        });
        let is_tram = railway.map_or(false, |r| r == "tram");
        let is_metro = railway.map_or(false, |r| r == "subway");

        let is_bus = if let Some(h) = highway {
            match h {
                "pedestrian" | "footway" | "steps" | "corridor" | "cycleway" | "path" | "track" => {
                    false
                }
                _ => true,
            }
        } else {
            false
        };

        (is_rail, is_tram, is_metro, is_bus)
    }

    fn parse_level(tags: &Tags) -> i32 {
        if let Some(l) = tags.get("layer") {
            l.parse().unwrap_or(0)
        } else {
            0
        }
    }

    fn parse_oneway(tags: &Tags) -> u8 {
        if let Some(ow) = tags.get("oneway") {
            if ow == "yes" || ow == "true" {
                return 1;
            }
            if ow == "-1" {
                return 2;
            }
        }
        0
    }

    fn get_speed(tags: &Tags) -> f64 {
        if let Some(ms) = tags.get("maxspeed") {
            if let Ok(v) = ms.parse::<f64>() {
                return v / 3.6;
            }
        }
        if let Some(h) = tags.get("highway") {
            match h.as_str() {
                "motorway" => 100.0 / 3.6,
                "trunk" => 80.0 / 3.6,
                "primary" => 70.0 / 3.6,
                "secondary" => 60.0 / 3.6,
                "tertiary" => 50.0 / 3.6,
                "residential" => 30.0 / 3.6,
                "living_street" => 10.0 / 3.6,
                "footway" | "pedestrian" => 4.0 / 3.6,
                _ => 50.0 / 3.6,
            }
        } else if let Some(r) = tags.get("railway") {
            match r.as_str() {
                "rail" => 100.0 / 3.6,
                "tram" => 40.0 / 3.6,
                "subway" => 80.0 / 3.6,
                _ => 50.0 / 3.6,
            }
        } else {
            10.0 // Slow
        }
    }

    fn calculate_cost(tags: &Tags, length_iters: f64) -> u32 {
        let speed = Self::get_speed(tags);
        let time_sec = length_iters / speed;
        let mut cost_float = time_sec * 10.0;

        // Apply penalties
        let penalty_factor = 100000.0;
        let mut penalized = false;

        let is_rail = tags.get("railway").map_or(false, |r| r == "rail");
        let is_industrial = tags.get("usage").map_or(false, |u| u == "industrial");

        // Extreme penalty for industrial rail
        if is_rail && is_industrial {
            cost_float *= 10.0 * penalty_factor;
        }

        if let Some(service) = tags.get("service") {
            if service.as_str() == "yard"
                || service.as_str() == "siding"
                || service.as_str() == "spur"
            {
                penalized = true;
            }
        }
        if let Some(usage) = tags.get("usage") {
            if usage.as_str() == "industrial" || usage.as_str() == "military" {
                penalized = true;
            }
        }

        if penalized {
            cost_float *= penalty_factor;
        }

        cost_float.min(u32::MAX as f64).ceil() as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osmpbfreader::Tags;

    #[test]
    fn test_calculate_cost_normal() {
        let mut tags = Tags::new();
        tags.insert("railway".into(), "rail".into());
        // Speed for rail is 100/3.6 approx 27.77 m/s
        // Length 100m -> time ~3.6s -> cost ~36
        let cost = OsmBuilder::calculate_cost(&tags, 100.0);
        assert!(cost > 30 && cost < 40);
    }

    #[test]
    fn test_calculate_cost_penalties() {
        let mut tags = Tags::new();
        tags.insert("railway".into(), "rail".into());
        tags.insert("service".into(), "yard".into());

        let cost_normal = {
            let mut t = Tags::new();
            t.insert("railway".into(), "rail".into());
            OsmBuilder::calculate_cost(&t, 100.0)
        };

        let cost_yard = OsmBuilder::calculate_cost(&tags, 100.0);
        assert!(cost_yard > cost_normal * 9); // Expect ~10x
    }

    #[test]
    fn test_calculate_cost_industrial() {
        let mut tags = Tags::new();
        tags.insert("railway".into(), "rail".into());
        tags.insert("usage".into(), "industrial".into());

        let cost_normal = {
            let mut t = Tags::new();
            t.insert("railway".into(), "rail".into());
            OsmBuilder::calculate_cost(&t, 100.0)
        };

        let cost_industrial = OsmBuilder::calculate_cost(&tags, 100.0);
        assert!(cost_industrial > cost_normal * 9);
    }

    #[test]
    fn test_is_infrastructure_platform() {
        let mut tags = Tags::new();
        tags.insert("railway".into(), "platform".into());

        let way = osmpbfreader::Way {
            id: osmpbfreader::WayId(1),
            tags: tags.clone(),
            nodes: vec![],
        };
        assert!(!OsmBuilder::is_infrastructure(&way));
        assert!(OsmBuilder::is_platform(&way));

        // Test mixed tags
        tags.insert("highway".into(), "footway".into());
        let way_mixed = osmpbfreader::Way {
            id: osmpbfreader::WayId(2),
            tags,
            nodes: vec![],
        };
        assert!(!OsmBuilder::is_infrastructure(&way_mixed));
        assert!(OsmBuilder::is_platform(&way_mixed));
    }

    #[test]
    fn test_is_platform_public_transport() {
        let mut tags = Tags::new();
        tags.insert("public_transport".into(), "platform".into());

        let way = osmpbfreader::Way {
            id: osmpbfreader::WayId(1),
            tags: tags.clone(),
            nodes: vec![],
        };
        assert!(OsmBuilder::is_platform(&way));
        assert!(!OsmBuilder::is_infrastructure(&way));
        assert!(OsmBuilder::is_platform(&way));
    }

    #[test]
    fn test_is_infrastructure_industrial() {
        let mut tags = Tags::new();
        tags.insert("railway".into(), "rail".into());
        tags.insert("usage".into(), "industrial".into());

        let way = osmpbfreader::Way {
            id: osmpbfreader::WayId(1),
            tags: tags.clone(),
            nodes: vec![],
        };
        // Should be false because of usage=industrial
        assert!(!OsmBuilder::is_infrastructure(&way));

        // Without industrial, it should be true
        let mut tags2 = Tags::new();
        tags2.insert("railway".into(), "rail".into());
        let way2 = osmpbfreader::Way {
            id: osmpbfreader::WayId(2),
            tags: tags2,
            nodes: vec![],
        };
        assert!(OsmBuilder::is_infrastructure(&way2));
    }

    #[test]
    fn test_calculate_cost_rail_industrial_extreme() {
        let mut tags = Tags::new();
        tags.insert("railway".into(), "rail".into());
        tags.insert("usage".into(), "industrial".into());

        // Normal cost ~36
        // Penalized once: ~36 * 100,000 = 3,600,000
        // Extreme penalty adds another 10x factor on top of the base penalty logic?
        // Actually looking at the code:
        // if is_rail && is_industrial { cost *= 10.0 * penalty_factor; } -> cost *= 1,000,000
        // Then later: if usage=industrial { penalized=true } -> cost *= penalty_factor (100,000)
        // Total multiplier = 1,000,000 * 100,000 = 10^11.
        // Base cost ~36. Total ~3.6 * 10^12.
        // Capped at u32::MAX (~4 * 10^9).
        // So we expect u32::MAX.

        let cost = OsmBuilder::calculate_cost(&tags, 100.0);
        assert_eq!(cost, u32::MAX);
    }
}

pub fn load_osm(
    path: &Path,
    used_route_types: &AHashSet<RouteType>,
    bbox: Option<(f64, f64, f64, f64)>,
) -> Result<OsmData> {
    OsmBuilder::read(path, used_route_types, bbox)
}
