use anyhow::Result;
use geo::{LineString, Point};
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader};
use rstar::RTree;
use std::collections::{HashMap, HashSet};
use std::path::Path;

use crate::graph::{EdgePL, Graph, NodeIndex, NodePL};

// RStar compatible struct for spatial indexing
#[derive(Debug, Clone, Copy)]
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
    pub tags: osmpbfreader::Tags,
    pub nodes: Vec<NodeIndex>, // All nodes in the relation, roughly ordered?
}

pub struct OsmData {
    pub graph: Graph<NodePL, EdgePL>,
    pub rail_tree: RTree<SpatialNode>,
    pub bus_tree: RTree<SpatialNode>, // Also includes tram, ferry, etc if general
    pub relations: Vec<OsmRelation>,
    pub node_to_relations: HashMap<NodeIndex, Vec<usize>>, // Map NodeIndex -> indices in self.relations
}

pub fn load_osm(path: &Path) -> Result<OsmData> {
    println!("Loading OSM from {:?}", path);
    let f = std::fs::File::open(path)?;
    let mut pbf = OsmPbfReader::new(f);

    // Read everything
    let objs = pbf.get_objs_and_deps(|obj| match obj {
        OsmObj::Way(w) => {
            if w.tags.contains_key("railway") {
                return true;
            }
            if let Some(h) = w.tags.get("highway") {
                let h_str = h.as_str();
                if h_str != "pedestrian" && h_str != "footway" {
                    return true;
                }
            }
            false
        }
        OsmObj::Relation(r) => {
            r.tags.contains_key("route") || r.tags.contains_key("public_transport")
        }
        _ => false,
    })?;

    println!("Loaded {} objects", objs.len());

    let mut graph = Graph::new();
    let mut osm_node_to_graph_index: HashMap<OsmId, NodeIndex> = HashMap::new();

    let mut rail_nodes: Vec<SpatialNode> = Vec::new();
    let mut bus_nodes: Vec<SpatialNode> = Vec::new();

    // Helper to intern nodes
    // Using a closure here is tricky with borrow checker and graph, so we just inline or separate logic
    // We need to keep track of which nodes are rail and which are bus.
    // A node can be both! (Example: level crossing or tram on street)
    // So we track sets of indices.
    let mut rail_node_indices: HashSet<NodeIndex> = HashSet::new();
    let mut bus_node_indices: HashSet<NodeIndex> = HashSet::new();
    // Also keep simplified mapping for spatial nodes creation at end? No, create on fly.
    // Use HashSet to avoid duplicate spatial nodes in tree bulk load? RTree bulk_load handles it?
    // Duplicate points in RTree is fine, but waste of memory.
    // Better: create separate lists.

    println!("Building Graph...");

    for (_id, obj) in &objs {
        if let OsmObj::Way(way) = obj {
            let is_rail = way.tags.contains_key("railway");
            let is_highway = way.tags.contains_key("highway");

            if !is_rail && !is_highway {
                continue;
            }

            // Need to reconstruct nodes from ID -> Coord
            let mut nodes_indices = Vec::new();
            let mut valid_way = true;

            for &node_id in &way.nodes {
                if let Some(OsmObj::Node(n)) = objs.get(&node_id.into()) {
                    let idx = if let Some(&idx) = osm_node_to_graph_index.get(&node_id.into()) {
                        idx
                    } else {
                        let point = Point::new(n.lon(), n.lat());
                        let idx = graph.add_node(NodePL { point });
                        osm_node_to_graph_index.insert(node_id.into(), idx);
                        idx
                    };
                    nodes_indices.push(idx);

                    if is_rail {
                        rail_node_indices.insert(idx);
                    }
                    if is_highway {
                        bus_node_indices.insert(idx);
                    }
                } else {
                    valid_way = false;
                    break;
                }
            }

            if valid_way && nodes_indices.len() > 1 {
                let mut edges_to_add = Vec::new();
                for i in 0..nodes_indices.len() - 1 {
                    let u = nodes_indices[i];
                    let v = nodes_indices[i + 1];

                    let p1 = graph.nodes[u].payload.point;
                    let p2 = graph.nodes[v].payload.point;

                    use geo::algorithm::VincentyDistance;
                    let weight = p1.vincenty_distance(&p2).unwrap_or(0.0);
                    let geom = LineString::new(vec![p1.into(), p2.into()]);
                    edges_to_add.push((u, v, geom, weight));
                }

                for (u, v, geom, weight) in edges_to_add {
                    graph.add_edge(
                        u,
                        v,
                        EdgePL {
                            geometry: geom,
                            weight,
                        },
                    );
                }
            }
        }
    }

    println!("Building Spatial Indices...");
    // Convert sets to Vec<SpatialNode>
    for idx in rail_node_indices {
        let p = graph.nodes[idx].payload.point;
        rail_nodes.push(SpatialNode {
            index: idx,
            point: [p.x(), p.y()],
        });
    }
    for idx in bus_node_indices {
        let p = graph.nodes[idx].payload.point;
        bus_nodes.push(SpatialNode {
            index: idx,
            point: [p.x(), p.y()],
        });
    }

    let rail_tree = RTree::bulk_load(rail_nodes);
    let bus_tree = RTree::bulk_load(bus_nodes);

    println!("Processing Relations...");
    let mut relations: Vec<OsmRelation> = Vec::new();
    let mut node_to_relations: HashMap<NodeIndex, Vec<usize>> = HashMap::new();

    for (id, obj) in &objs {
        if let OsmObj::Relation(r) = obj {
            // Flatten members to nodes
            let mut relation_nodes: Vec<NodeIndex> = Vec::new();

            for member in &r.refs {
                let member_id = member.member;
                match member_id {
                    OsmId::Node(nid) => {
                        if let Some(&idx) = osm_node_to_graph_index.get(&OsmId::Node(nid)) {
                            relation_nodes.push(idx);
                        }
                    }
                    OsmId::Way(wid) => {
                        // Find way in objs
                        if let Some(OsmObj::Way(w)) = objs.get(&OsmId::Way(wid)) {
                            for nid in &w.nodes {
                                if let Some(&idx) = osm_node_to_graph_index.get(&OsmId::Node(*nid))
                                {
                                    relation_nodes.push(idx);
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }

            if !relation_nodes.is_empty() {
                let rel_idx = relations.len();
                // Map nodes back to this relation
                for &n_idx in &relation_nodes {
                    node_to_relations.entry(n_idx).or_default().push(rel_idx);
                }

                relations.push(OsmRelation {
                    id: id.inner_id(),
                    tags: r.tags.clone(),
                    nodes: relation_nodes,
                });
            }
        }
    }

    println!(
        "Graph built: {} nodes, {} edges. Relations: {}",
        graph.nodes.len(),
        graph.edges.len(),
        relations.len()
    );

    Ok(OsmData {
        graph,
        rail_tree,
        bus_tree,
        relations,
        node_to_relations,
    })
}
