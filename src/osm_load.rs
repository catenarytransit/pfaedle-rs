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
        println!("Reading OSM file {:?} ...", path);
        let f = std::fs::File::open(path).with_context(|| format!("Failed to open {:?}", path))?;
        let mut pbf = OsmPbfReader::new(f);

        let objs = pbf
            .get_objs_and_deps(|obj| if Self::keep_obj(obj) { true } else { false })
            .context("Failed to read OSM PBF")?;

        println!("Loaded {} objects. Processing...", objs.len());

        // Step 1: Process Relations to map Ways -> TransitLines
        let mut way_rels: HashMap<OsmId, Vec<OsmId>> = HashMap::new();
        // let mut node_rels: HashMap<OsmId, Vec<OsmId>> = HashMap::new();
        let mut relations_list: Vec<OsmRelation> = Vec::new(); // For final output

        for (id, obj) in &objs {
            if let OsmObj::Relation(r) = obj {
                for member in &r.refs {
                    match member.member {
                        OsmId::Way(w) => way_rels.entry(OsmId::Way(w)).or_default().push(*id),
                        // OsmId::Node(n) => node_rels.entry(OsmId::Node(n)).or_default().push(*id),
                        _ => {}
                    }
                }
            }
        }

        // Step 2: Build Graph from Ways
        let mut graph = Graph::new();
        let mut osm_node_to_graph_idx: HashMap<OsmId, NodeIndex> = HashMap::new();

        let mut rail_node_indices: HashSet<NodeIndex> = HashSet::new();
        let mut tram_node_indices: HashSet<NodeIndex> = HashSet::new();
        let mut metro_node_indices: HashSet<NodeIndex> = HashSet::new();
        let mut bus_node_indices: HashSet<NodeIndex> = HashSet::new();

        // Pass 3 (Edges) logic
        for (id, obj) in &objs {
            if let OsmObj::Way(w) = obj {
                if !Self::is_valid_way(w) {
                    continue;
                }

                let is_route_member = way_rels.contains_key(id);
                if !Self::is_infrastructure(w) && !is_route_member {
                    continue;
                }

                let (is_rail, is_tram, is_metro, is_bus) = Self::classify_way(w);
                if !is_rail && !is_tram && !is_metro && !is_bus {
                    if !is_route_member {
                        continue;
                    }
                }

                let transit_lines = if let Some(rel_ids) = way_rels.get(id) {
                    Self::get_lines(rel_ids, &objs)
                } else {
                    Vec::new()
                };

                let mut way_node_indices = Vec::new();
                for nid in &w.nodes {
                    let osm_nid = OsmId::Node(*nid);
                    if let Some(OsmObj::Node(n)) = objs.get(&osm_nid) {
                        let idx = *osm_node_to_graph_idx.entry(osm_nid).or_insert_with(|| {
                            graph.add_node(NodePL {
                                point: Point::new(n.lon(), n.lat()),
                            })
                        });
                        way_node_indices.push(idx);

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

                if way_node_indices.len() > 1 {
                    for i in 0..way_node_indices.len() - 1 {
                        let u = way_node_indices[i];
                        let v = way_node_indices[i + 1];

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

        Self::post_process(&mut graph);

        // Build Output Relations
        let mut final_node_to_rels: HashMap<NodeIndex, Vec<usize>> = HashMap::new();

        for (id, obj) in &objs {
            if let OsmObj::Relation(r) = obj {
                if r.tags.get("type").map(|s| s.as_str()) != Some("route") {
                    continue;
                }

                let rel_idx = relations_list.len();
                let mut rel_nodes = Vec::new();

                for member in &r.refs {
                    match member.member {
                        OsmId::Node(nid) => {
                            if let Some(&idx) = osm_node_to_graph_idx.get(&OsmId::Node(nid)) {
                                rel_nodes.push(idx);
                                final_node_to_rels.entry(idx).or_default().push(rel_idx);
                            }
                        }
                        OsmId::Way(wid) => {
                            if let Some(OsmObj::Way(w)) = objs.get(&OsmId::Way(wid)) {
                                for nid in &w.nodes {
                                    if let Some(&idx) =
                                        osm_node_to_graph_idx.get(&OsmId::Node(*nid))
                                    {
                                        rel_nodes.push(idx);
                                        final_node_to_rels.entry(idx).or_default().push(rel_idx);
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                }

                relations_list.push(OsmRelation {
                    id: id.inner_id(),
                    tags: r.tags.clone(),
                    nodes: rel_nodes,
                });
            }
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

    fn keep_obj(obj: &OsmObj) -> bool {
        match obj {
            OsmObj::Way(w) => {
                if w.tags.contains_key("railway") {
                    return true;
                }
                if let Some(h) = w.tags.get("highway") {
                    let h_str = h.as_str();
                    if matches!(h_str, "steps" | "corridor" | "cycleway" | "track") {
                        return true;
                    }
                    return true;
                }
                false
            }
            OsmObj::Relation(r) => {
                r.tags.contains_key("route") || r.tags.contains_key("public_transport")
            }
            OsmObj::Node(_) => true,
        }
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

    fn get_lines(
        rel_ids: &[OsmId],
        objs: &std::collections::BTreeMap<OsmId, OsmObj>,
    ) -> Vec<TransitInfo> {
        let mut lines = Vec::new();
        for rid in rel_ids {
            if let Some(OsmObj::Relation(r)) = objs.get(rid) {
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
                    lines.push(TransitInfo {
                        short_name,
                        from_str,
                        to_str,
                    });
                }
            }
        }
        lines.sort();
        lines.dedup();
        lines
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
