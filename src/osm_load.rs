use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use geo::{LineString, Point};
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader, Tags};
use rayon::prelude::*; // Make sure rayon is available for parallel sort
use rstar::RTree;

use std::path::Path;

use crate::graph::{
    EdgeIndex, EdgePL, Graph, MODE_BUS, MODE_FERRY, MODE_GONDOLA, MODE_RAIL, MODE_SUBWAY,
    MODE_TRAM, NodeIndex, NodePL, TransitInfo,
};
use gtfs_structures::RouteType;

// RStar compatible struct for spatial indexing
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SpatialNode {
    pub index: NodeIndex,
    pub point: [f64; 2], // [x, y] = [lon, lat]
    pub modes: u8,       // Bitmask of allowed modes
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
    /// Infrastructure way geometries in relation-member order. Each inner vec
    /// preserves the OSM way's native node order; the matcher may reverse an
    /// individual way when stitching the route, but never reorders the ways.
    pub member_ways: Vec<Vec<NodeIndex>>,
}

#[derive(Clone)]
pub struct OsmData {
    pub graph: Graph<NodePL, EdgePL>,
    pub timestamp: String,
    pub spatial_tree: Option<RTree<SpatialNode>>,
    pub osm_filepath: std::path::PathBuf,
    pub relations: Vec<OsmRelation>,
    pub node_to_relations: AHashMap<NodeIndex, Vec<usize>>,
}

/// Lightweight relation data for color matching without full graph.
#[derive(Debug, Clone)]
pub struct LightRelation {
    pub id: i64,
    pub ref_tag: Option<String>,
    pub name: Option<String>,
    pub operator: Option<String>,
    pub colour: Option<String>,
    pub route_type: Option<String>,
    pub way_ids: Vec<i64>,
}

/// Lightweight OSM data containing only relations (for color matching).
/// This is used for bus processing to avoid loading the full graph.
#[derive(Debug, Clone)]
pub struct LightOsmData {
    pub relations: Vec<LightRelation>,
    /// Map from way_id -> indices into `relations` vec
    pub way_to_relations: AHashMap<i64, Vec<usize>>,
}

impl LightOsmData {
    pub fn new() -> Self {
        Self {
            relations: Vec::new(),
            way_to_relations: AHashMap::new(),
        }
    }

    /// Find relations that contain a given way
    pub fn relations_for_way(&self, way_id: i64) -> impl Iterator<Item = &LightRelation> {
        self.way_to_relations
            .get(&way_id)
            .into_iter()
            .flat_map(|indices| indices.iter().map(|&i| &self.relations[i]))
    }

    /// Find color for a route matching the given criteria
    pub fn find_color(
        &self,
        route_short_name: Option<&str>,
        route_long_name: Option<&str>,
        operator: Option<&str>,
    ) -> Option<String> {
        let mut best_match: Option<(&LightRelation, u8)> = None;

        for rel in &self.relations {
            if rel.colour.is_none() {
                continue;
            }

            let mut score: u8 = 0;

            // Check ref/name match
            if let Some(osm_ref) = &rel.ref_tag {
                let osm_lower = osm_ref.to_lowercase();
                if let Some(short) = route_short_name {
                    if osm_lower.contains(short) || short.contains(&osm_lower) {
                        score += 3;
                    }
                }
            }
            if let Some(osm_name) = &rel.name {
                let osm_lower = osm_name.to_lowercase();
                if let Some(short) = route_short_name {
                    if osm_lower.contains(short) || short.contains(&osm_lower) {
                        score += 2;
                    }
                }
                if let Some(long) = route_long_name {
                    if osm_lower.contains(long) || long.contains(&osm_lower) {
                        score += 2;
                    }
                }
            }

            // Check operator match
            if let (Some(osm_op), Some(gtfs_op)) = (&rel.operator, operator) {
                let osm_lower = osm_op.to_lowercase();
                if osm_lower.contains(gtfs_op) || gtfs_op.contains(&osm_lower) {
                    score += 1;
                }
            }

            if score > 0 {
                if best_match.is_none() || score > best_match.as_ref().unwrap().1 {
                    best_match = Some((rel, score));
                }
            }
        }

        best_match.and_then(|(rel, _)| rel.colour.clone())
    }
}

pub struct OsmBuilder;

// Relation metadata found in Pass 1
#[derive(Debug, Clone)]
pub struct PreRelation {
    pub id: i64,
    pub tags: Tags,
    pub members: Vec<osmpbfreader::Ref>,
}

impl OsmBuilder {
    pub fn identify_resources(
        path: &Path,
        skip_small_roads: bool,
    ) -> Result<(Vec<PreRelation>, AHashSet<i64>, AHashSet<i64>, Vec<i64>)> {
        let mut pre_relations: Vec<PreRelation> = Vec::new();
        let mut ways_in_relations: AHashSet<i64> = AHashSet::new();
        let mut ways_in_ferry_relations: AHashSet<i64> = AHashSet::new();
        let mut needed_nodes: Vec<i64> = Vec::new();

        println!("Pass 1/4: Scanning relations...");
        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 1")?;
                if let OsmObj::Relation(r) = obj {
                    let is_route =
                        r.tags.contains_key("route") || r.tags.contains_key("public_transport");
                    if is_route {
                        for member in &r.refs {
                            if let OsmId::Way(wid) = member.member {
                                ways_in_relations.insert(wid.0);
                                if r.tags.get("route").map_or(false, |s| s == "ferry") {
                                    ways_in_ferry_relations.insert(wid.0);
                                }
                            } else if let OsmId::Node(nid) = member.member {
                                needed_nodes.push(nid.0);
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

        println!("Pass 2/4: Scanning ways to identify needed nodes...");
        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 2")?;
                if let OsmObj::Way(w) = obj {
                    let wid = w.id.0;
                    let is_infra =
                        Self::is_infrastructure(&w) || ways_in_ferry_relations.contains(&wid);
                    let is_platform = Self::is_platform(&w);
                    let is_rel_member = ways_in_relations.contains(&wid) && !is_platform;

                    if skip_small_roads && is_infra && !is_rel_member {
                        let highway = w.tags.get("highway").map(|s| s.as_str());
                        let service = w.tags.get("service").map(|s| s.as_str());
                        if highway == Some("residential")
                            || highway == Some("service")
                            || service == Some("driveway")
                        {
                            continue;
                        }
                    }

                    if is_infra || is_rel_member {
                        if !Self::is_valid_way(&w) {
                            continue;
                        }
                        for nid in &w.nodes {
                            needed_nodes.push(nid.0);
                        }
                    }
                }
            }
        }

        println!("  Sorting {} needed nodes...", needed_nodes.len());
        needed_nodes.par_sort_unstable();
        needed_nodes.dedup();
        println!(
            "  Identified {} unique nodes needed for the graph.",
            needed_nodes.len()
        );

        Ok((
            pre_relations,
            ways_in_relations,
            ways_in_ferry_relations,
            needed_nodes,
        ))
    }

    pub fn read(
        path: &Path,
        used_route_types: &AHashSet<RouteType>,
        bbox: Option<(f64, f64, f64, f64)>, // min_lon, min_lat, max_lon, max_lat
        skip_small_roads: bool,
    ) -> Result<OsmData> {
        println!("Reading OSM file {:?} in multiple passes...", path);

        let (pre_relations, ways_in_relations, ways_in_ferry_relations, mut needed_nodes) =
            Self::identify_resources(path, skip_small_roads)?;

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
        let mut ferry_node_indices: AHashSet<NodeIndex> = AHashSet::new();
        let mut gondola_node_indices: AHashSet<NodeIndex> = AHashSet::new();
        let mut stop_node_indices: AHashSet<NodeIndex> = AHashSet::new();

        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 3")?;
                if let OsmObj::Node(n) = obj {
                    let nid = n.id.0;
                    if needed_nodes.binary_search(&nid).is_ok() {
                        let lat = n.lat();
                        let lon = n.lon();

                        // Filter by Bounding Box if present
                        if let Some((min_lon, min_lat, max_lon, max_lat)) = bbox {
                            if lon < min_lon || lon > max_lon || lat < min_lat || lat > max_lat {
                                continue;
                            }
                        }

                        let idx = graph.add_node(NodePL {
                            comp_id: 0,
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
            let operator = r.tags.get("operator").map(|s| s.to_string());

            if !short_name.is_empty() || !from_str.is_empty() {
                Some(TransitInfo {
                    short_name,
                    from_str,
                    to_str,
                    operator,
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
                    let is_infra =
                        Self::is_infrastructure(&w) || ways_in_ferry_relations.contains(&wid);

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
                    let (is_rail, is_tram, is_metro, is_bus, is_ferry, is_gondola) =
                        Self::classify_way(&w, &ways_in_ferry_relations);

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
                            if is_ferry {
                                ferry_node_indices.insert(idx);
                            }
                            if is_gondola {
                                gondola_node_indices.insert(idx);
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
                            edge_pl.preferred_direction = Self::parse_preferred_direction(&w.tags);

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
                            if is_ferry {
                                modes |= MODE_FERRY;
                            }
                            if is_gondola {
                                modes |= MODE_GONDOLA;
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
                out_member_ways: &mut Vec<Vec<NodeIndex>>,
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
                                    if nodes.len() >= 2 {
                                        out_member_ways.push(nodes.clone());
                                    }
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
                                    out_member_ways,
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
            let mut rel_member_ways = Vec::new();
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
                &mut rel_member_ways,
                &mut final_node_to_rels,
                rel_idx,
            );

            relations_list.push(OsmRelation {
                id: r_pre.id,
                tags: r_pre.tags.clone(),
                nodes: rel_nodes,
                edges: rel_edges,
                member_ways: rel_member_ways,
            });
        }

        // Consolidate all nodes into one Spatial Tree with mode masks
        println!("Building unified spatial index...");
        let mut node_modes: AHashMap<NodeIndex, u8> = AHashMap::new();

        let needs_rail = used_route_types.contains(&RouteType::Rail);
        let needs_tram = used_route_types.contains(&RouteType::Tramway);
        let needs_metro = used_route_types.contains(&RouteType::Subway);
        let needs_bus = used_route_types.iter().any(|r| {
            *r == RouteType::Bus
                || (*r != RouteType::Rail
                    && *r != RouteType::Tramway
                    && *r != RouteType::Subway
                    && *r != RouteType::Ferry
                    && *r != RouteType::Gondola)
        });
        let needs_ferry = used_route_types.contains(&RouteType::Ferry);
        let needs_gondola = used_route_types.contains(&RouteType::Gondola);

        let mut add_modes = |indices: &AHashSet<NodeIndex>, mode_flag: u8| {
            for &idx in indices {
                if !stop_node_indices.contains(&idx) {
                    *node_modes.entry(idx).or_default() |= mode_flag;
                }
            }
        };

        if needs_rail {
            add_modes(&rail_node_indices, MODE_RAIL);
        }
        if needs_tram {
            add_modes(&tram_node_indices, MODE_TRAM);
        }
        if needs_metro {
            add_modes(&metro_node_indices, MODE_SUBWAY);
        }
        if needs_bus {
            add_modes(&bus_node_indices, MODE_BUS);
        }
        if needs_ferry {
            add_modes(&ferry_node_indices, MODE_FERRY);
        }
        if needs_gondola {
            add_modes(&gondola_node_indices, MODE_GONDOLA);
        }

        // Also ensure we include nodes that might be used for generic road access (Bus covers most)
        // But the sets above only include nodes from classifying ways.

        let mut spatial_nodes: Vec<SpatialNode> = Vec::with_capacity(node_modes.len());
        for (idx, modes) in node_modes {
            let p = graph.nodes[idx].payload.point;
            spatial_nodes.push(SpatialNode {
                index: idx,
                point: [p.x(), p.y()],
                modes,
            });
        }

        let spatial_tree = if !spatial_nodes.is_empty() {
            Some(RTree::bulk_load(spatial_nodes))
        } else {
            None
        };

        println!(
            "Graph built: {} nodes, {} edges. Relations: {}. Spatial Index size: {}",
            graph.nodes.len(),
            graph.edges.len(),
            relations_list.len(),
            if let Some(t) = &spatial_tree {
                t.size()
            } else {
                0
            }
        );

        Ok(OsmData {
            graph,
            timestamp: "unknown".to_string(),
            spatial_tree,
            osm_filepath: path.to_path_buf(),
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
        for edge in &mut graph.edges {
            // Apply preferred direction penalties to the forward edge
            // 0: Neutral, 1: Forward (benefit), 2: Backward (penalty)
            if edge.payload.preferred_direction == 1 {
                // Forward is preferred: slight benefit (0.9x)
                edge.payload.cost = (edge.payload.cost as f64 * 0.9).round() as u32;
            } else if edge.payload.preferred_direction == 2 {
                // Backward is preferred: slight penalty (1.2x) for forward traversal
                edge.payload.cost = (edge.payload.cost as f64 * 1.2).round() as u32;
            }

            let u = edge.from;
            let v = edge.to;
            if !existing_edges.contains(&(v, u)) {
                let mut rev_pl = edge.payload.rev_copy();

                // Logic for reverse edge cost based on preferred direction
                // Original edge had preferred_direction:
                // 1 (Forward): Reverse edge is AGAINST preference -> Penalty (1.2x)
                // 2 (Backward): Reverse edge is WITH preference -> Benefit (0.9x)

                let mut base_cost = edge.payload.cost as f64;
                if edge.payload.preferred_direction == 1 {
                    base_cost /= 0.9;
                } else if edge.payload.preferred_direction == 2 {
                    base_cost /= 1.2;
                }

                if edge.payload.preferred_direction == 1 {
                    // Reverse is against preference
                    rev_pl.cost = (base_cost * 1.2).round() as u32;
                } else if edge.payload.preferred_direction == 2 {
                    // Reverse is with preference
                    rev_pl.cost = (base_cost * 0.9).round() as u32;
                } else {
                    // No preference, apply standard 10% penalty for reverse
                    rev_pl.cost = (base_cost * 1.1).round() as u32;
                }

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

        // Compute connected components
        let mut comp_id = 0;
        let mut visited = vec![false; graph.nodes.len()];
        let mut queue = std::collections::VecDeque::new();
        for i in 0..graph.nodes.len() {
            if !visited[i] {
                comp_id += 1;
                queue.push_back(i);
                visited[i] = true;

                while let Some(u) = queue.pop_front() {
                    graph.nodes[u].payload.comp_id = comp_id;

                    for &edge_idx in &graph.nodes[u].out_edges {
                        let v = graph.edges[edge_idx].to;
                        if !visited[v] {
                            visited[v] = true;
                            queue.push_back(v);
                        }
                    }
                    for &edge_idx in &graph.nodes[u].in_edges {
                        let v = graph.edges[edge_idx].from;
                        if !visited[v] {
                            visited[v] = true;
                            queue.push_back(v);
                        }
                    }
                }
            }
        }
    }

    fn has_tag(tags: &Tags, key: &str, val: &str) -> bool {
        tags.get(key).map(|s| s.as_str()) == Some(val)
    }

    pub fn is_valid_way(w: &osmpbfreader::Way) -> bool {
        w.nodes.len() > 1
    }

    pub fn is_infrastructure(w: &osmpbfreader::Way) -> bool {
        if Self::is_platform(w) {
            return false;
        }
        // Filter out industrial usage
        if w.tags.get("usage").map_or(false, |u| u == "industrial") {
            return false;
        }

        // Critical Fix: Explicitly include route=ferry ways even if they lack highway tags
        if w.tags.get("route").map_or(false, |r| r == "ferry") {
            return true;
        }
        // Critical Fix: Explicitly include aerialway ways for gondolas
        if w.tags.contains_key("aerialway") {
            return true;
        }

        w.tags.contains_key("railway") || w.tags.contains_key("highway")
    }

    pub fn is_platform(w: &osmpbfreader::Way) -> bool {
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

    pub fn classify_way(
        w: &osmpbfreader::Way,
        ferry_ways: &AHashSet<i64>,
    ) -> (bool, bool, bool, bool, bool, bool) {
        let railway = w.tags.get("railway").map(|s| s.as_str());
        let highway = w.tags.get("highway").map(|s| s.as_str());
        let route = w.tags.get("route").map(|s| s.as_str());
        let aerialway = w.tags.get("aerialway").map(|s| s.as_str());

        let is_rail = railway.map_or(false, |r| {
            r == "rail" || r == "light_rail" || r == "narrow_gauge"
        });
        let is_tram = railway.map_or(false, |r| r == "tram");
        let is_metro = railway.map_or(false, |r| r == "subway");

        // Ferry detection
        let is_ferry = route == Some("ferry") || ferry_ways.contains(&w.id.0);

        // Gondola detection
        let is_gondola = aerialway.is_some();

        let is_bus = if is_ferry || is_gondola {
            false
        } else if let Some(h) = highway {
            match h {
                "pedestrian" | "footway" | "steps" | "corridor" | "cycleway" | "path" | "track" => {
                    false
                }
                _ => true,
            }
        } else {
            false
        };

        (is_rail, is_tram, is_metro, is_bus, is_ferry, is_gondola)
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

    fn parse_preferred_direction(tags: &Tags) -> u8 {
        if let Some(pd) = tags.get("railway:preferred_direction") {
            match pd.as_str() {
                "forward" => 1,
                "backward" => 2,
                _ => 0,
            }
        } else {
            0
        }
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
        } else if let Some(r) = tags.get("route") {
            if r == "ferry" { 15.0 / 3.6 } else { 10.0 }
        } else if tags.contains_key("aerialway") {
            15.0 / 3.6 // Gondola speed approx
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
    pub fn read_relations_only(path: &Path) -> Result<LightOsmData> {
        println!("Reading OSM relations only (light pass) from {:?}...", path);
        let mut pbf = Self::open_pbf(path)?;

        let mut relations = Vec::new();
        let mut way_to_relations: AHashMap<i64, Vec<usize>> = AHashMap::new();

        for obj in pbf.iter() {
            let obj = obj.context("Error reading PBF object")?;
            if let OsmObj::Relation(r) = obj {
                let is_route =
                    r.tags.contains_key("route") || r.tags.contains_key("public_transport");
                if is_route {
                    let id = r.id.0;
                    let ref_tag = r.tags.get("ref").map(|s| s.to_string());
                    let name = r.tags.get("name").map(|s| s.to_string());
                    let operator = r.tags.get("operator").map(|s| s.to_string());
                    let colour = r.tags.get("colour").map(|s| s.to_string());
                    let route_type = r.tags.get("route").map(|s| s.to_string());

                    let mut way_ids = Vec::new();
                    for member in &r.refs {
                        if let OsmId::Way(wid) = member.member {
                            way_ids.push(wid.0);
                        }
                    }

                    let rel_idx = relations.len();
                    for &wid in &way_ids {
                        way_to_relations.entry(wid).or_default().push(rel_idx);
                    }

                    relations.push(LightRelation {
                        id,
                        ref_tag,
                        name,
                        operator,
                        colour,
                        route_type,
                        way_ids,
                    });
                }
            }
        }

        println!("Loaded {} relations for light matching.", relations.len());

        Ok(LightOsmData {
            relations,
            way_to_relations,
        })
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
    #[test]
    fn test_preferred_direction_costs() {
        let mut graph = Graph::new();
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n2 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.1, 0.0),
        });

        // Add an edge with preferred_direction: forward (1)
        let mut edge_pl = EdgePL::new();
        edge_pl.geometry = LineString::new(vec![
            graph.nodes[n1].payload.point.into(),
            graph.nodes[n2].payload.point.into(),
        ]);
        edge_pl.cost = 100;
        edge_pl.preferred_direction = 1; // Forward
        edge_pl.oneway = 0; // Bidirectional

        graph.add_edge(n1, n2, edge_pl);

        // Run post_process to apply costs and generate reverse edges
        OsmBuilder::post_process(&mut graph);

        // Check forward edge (index 0)
        let fwd_edge = &graph.edges[0];
        // Expected: 100 * 0.9 = 90
        assert_eq!(fwd_edge.payload.cost, 90);

        // Check reverse edge (index 1)
        // Expected: 100 * 1.2 = 120
        let rev_edge = &graph.edges[1];
        assert_eq!(rev_edge.payload.cost, 120);
    }

    #[test]
    fn test_preferred_direction_backward() {
        let mut graph = Graph::new();
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n2 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.1, 0.0),
        });

        // Add an edge with preferred_direction: backward (2)
        let mut edge_pl = EdgePL::new();
        edge_pl.geometry = LineString::new(vec![
            graph.nodes[n1].payload.point.into(),
            graph.nodes[n2].payload.point.into(),
        ]);
        edge_pl.cost = 100;
        edge_pl.preferred_direction = 2; // Backward
        edge_pl.oneway = 0;

        graph.add_edge(n1, n2, edge_pl);

        OsmBuilder::post_process(&mut graph);

        // Check forward edge (index 0)
        // Expected: 100 * 1.2 = 120
        let fwd_edge = &graph.edges[0];
        assert_eq!(fwd_edge.payload.cost, 120);

        // Check reverse edge (index 1)
        // Logic:
        // if pref == 2: base_cost /= 1.2;
        // rev_pl.cost = (base_cost * 0.9) as u32;
        // base_cost = 120 / 1.2 = 100
        // rev_cost = 100 * 0.9 = 90
        let rev_edge = &graph.edges[1];
        assert_eq!(rev_edge.payload.cost, 90);
    }
}

pub fn load_osm(
    path: &Path,
    used_route_types: &AHashSet<RouteType>,
    bbox: Option<(f64, f64, f64, f64)>,
    skip_small_roads: bool,
) -> Result<OsmData> {
    OsmBuilder::read(path, used_route_types, bbox, skip_small_roads)
}
