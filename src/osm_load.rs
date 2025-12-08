use anyhow::{Context, Result};
use geo::{LineString, Point};
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader, Tags};
use rstar::RTree;
use std::collections::{HashMap, HashSet};
use std::path::Path;

use crate::graph::{EdgePL, Graph, NodeIndex, NodePL, TransitInfo};
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
    pub nodes: Vec<NodeIndex>, // All nodes in the relation
}

pub struct OsmData {
    pub graph: Graph<NodePL, EdgePL>,
    pub rail_tree: Option<RTree<SpatialNode>>,
    pub tram_tree: Option<RTree<SpatialNode>>,
    pub metro_tree: Option<RTree<SpatialNode>>,
    pub bus_tree: Option<RTree<SpatialNode>>,
    pub relations: Vec<OsmRelation>,
    pub node_to_relations: HashMap<NodeIndex, Vec<usize>>,
}

pub struct OsmBuilder;

impl OsmBuilder {
    pub fn read(path: &Path, used_route_types: &HashSet<RouteType>) -> Result<OsmData> {
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
        let mut ways_in_relations: HashSet<i64> = HashSet::new();

        // Set of Node IDs that are required for the graph.
        let mut needed_nodes: HashSet<i64> = HashSet::new();

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
                    let is_rel_member = ways_in_relations.contains(&wid);

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
        let mut osm_node_to_graph_idx: HashMap<i64, NodeIndex> = HashMap::new();

        let mut rail_node_indices: HashSet<NodeIndex> = HashSet::new();
        let mut tram_node_indices: HashSet<NodeIndex> = HashSet::new();
        let mut metro_node_indices: HashSet<NodeIndex> = HashSet::new();
        let mut bus_node_indices: HashSet<NodeIndex> = HashSet::new();

        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 3")?;
                if let OsmObj::Node(n) = obj {
                    let nid = n.id.0;
                    if needed_nodes.contains(&nid) {
                        let idx = graph.add_node(NodePL {
                            point: Point::new(n.lon(), n.lat()),
                        });
                        osm_node_to_graph_idx.insert(nid, idx);
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
        let mut way_id_to_node_indices: HashMap<i64, Vec<NodeIndex>> = HashMap::new();

        // 4a. Build Way -> TransitInfo lookup
        let mut way_transit_info: HashMap<i64, Vec<TransitInfo>> = HashMap::new();

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
                    let is_rel_member = ways_in_relations.contains(&wid);
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

                            let speed = Self::get_speed(&w.tags);
                            let len = edge_pl.length();
                            let time_sec = len / speed;
                            edge_pl.cost = (time_sec * 10.0).ceil() as u32;

                            graph.add_edge(u, v, edge_pl);
                        }
                    }
                }
            }
        }

        Self::post_process(&mut graph);

        // Build Output Relations
        let mut final_node_to_rels: HashMap<NodeIndex, Vec<usize>> = HashMap::new();
        let mut relations_list: Vec<OsmRelation> = Vec::new();

        for r_pre in pre_relations {
            if r_pre.tags.get("type").map(|s| s.as_str()) != Some("route") {
                continue;
            }

            let rel_idx = relations_list.len();
            let mut rel_nodes = Vec::new();

            for member in &r_pre.members {
                match member.member {
                    OsmId::Node(nid) => {
                        if let Some(&idx) = osm_node_to_graph_idx.get(&nid.0) {
                            rel_nodes.push(idx);
                            final_node_to_rels.entry(idx).or_default().push(rel_idx);
                        }
                    }
                    OsmId::Way(wid) => {
                        if let Some(nodes) = way_id_to_node_indices.get(&wid.0) {
                            for &idx in nodes {
                                rel_nodes.push(idx);
                                final_node_to_rels.entry(idx).or_default().push(rel_idx);
                            }
                        }
                    }
                    _ => {}
                }
            }

            relations_list.push(OsmRelation {
                id: r_pre.id,
                tags: r_pre.tags,
                nodes: rel_nodes,
            });
        }

        let build_tree = |indices: HashSet<NodeIndex>, name: &str| -> Option<RTree<SpatialNode>> {
            if indices.is_empty() {
                return None;
            }
            println!("Building {} index with {} nodes...", name, indices.len());
            let nodes: Vec<SpatialNode> = indices
                .iter()
                .map(|&idx| {
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
        let existing_edges: HashSet<(NodeIndex, NodeIndex)> =
            graph.edges.iter().map(|e| (e.from, e.to)).collect();

        let mut edges_to_add = Vec::new();
        for edge in &graph.edges {
            let u = edge.from;
            let v = edge.to;
            if !existing_edges.contains(&(v, u)) {
                let rev_pl = edge.payload.rev_copy();
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
        w.tags.contains_key("railway") || w.tags.contains_key("highway")
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
}

pub fn load_osm(path: &Path, used_route_types: &HashSet<RouteType>) -> Result<OsmData> {
    OsmBuilder::read(path, used_route_types)
}
