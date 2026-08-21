use ahash::AHashMap;
use geo::{HaversineLength, LineString, Point};
use serde::{Deserialize, Serialize};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

pub type NodeIndex = usize;
pub type EdgeIndex = usize;
pub type TransitInfoId = u32;

pub const MODE_RAIL: u8 = 1;
pub const MODE_TRAM: u8 = 2;
pub const MODE_SUBWAY: u8 = 4;
pub const MODE_BUS: u8 = 8;
pub const MODE_FERRY: u8 = 16;
pub const MODE_GONDOLA: u8 = 32;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node<N> {
    pub payload: N,
    pub out_edges: Vec<EdgeIndex>,
    pub in_edges: Vec<EdgeIndex>,
}

impl<N> Node<N> {
    pub fn edges(&self) -> impl Iterator<Item = &EdgeIndex> {
        self.out_edges.iter().chain(self.in_edges.iter())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge<E> {
    pub payload: E,
    pub from: NodeIndex,
    pub to: NodeIndex,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Graph<N, E> {
    pub nodes: Vec<Node<N>>,
    pub edges: Vec<Edge<E>>,
    /// Canonical transit-line metadata referenced by `EdgePL::lines`.
    ///
    /// Upstream pfaedle stores a vector of shared `TransitEdgeLine*` on every
    /// edge. Rust uses compact IDs instead: a line's strings are owned exactly
    /// once by the graph and edges only retain `u32` IDs.
    #[serde(default)]
    pub transit_lines: Vec<TransitInfo>,
}

impl<N, E> Graph<N, E> {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            transit_lines: Vec::new(),
        }
    }

    pub fn add_node(&mut self, payload: N) -> NodeIndex {
        let index = self.nodes.len();
        self.nodes.push(Node {
            payload,
            out_edges: Vec::new(),
            in_edges: Vec::new(),
        });
        index
    }

    pub fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, payload: E) -> EdgeIndex {
        let index = self.edges.len();
        self.edges.push(Edge { payload, from, to });

        self.nodes[from].out_edges.push(index);
        self.nodes[to].in_edges.push(index);

        index
    }

    pub fn node(&self, index: NodeIndex) -> &Node<N> {
        &self.nodes[index]
    }

    pub fn edge(&self, index: EdgeIndex) -> &Edge<E> {
        &self.edges[index]
    }

    pub fn transit_info(&self, id: TransitInfoId) -> Option<&TransitInfo> {
        self.transit_lines.get(id as usize)
    }
}

// Payloads corresponding to pfaedle's netgraph

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodePL {
    pub point: Point<f64>,
    #[serde(default)]
    pub comp_id: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TransitInfo {
    pub short_name: String,
    pub from_str: String,
    pub to_str: String,
}

/// Build-time interner for transit-line metadata.
///
/// The hash table stores only hashes and compact IDs, rather than cloning the
/// line strings into the lookup table. Once graph construction is finished the
/// interner itself is dropped and only the canonical `Vec<TransitInfo>` remains.
#[derive(Debug, Default)]
pub struct TransitInfoInterner {
    infos: Vec<TransitInfo>,
    buckets: AHashMap<u64, Vec<TransitInfoId>>,
}

impl TransitInfoInterner {
    pub fn new() -> Self {
        Self::default()
    }

    fn hash_info(info: &TransitInfo) -> u64 {
        let mut hasher = DefaultHasher::new();
        info.hash(&mut hasher);
        hasher.finish()
    }

    pub fn intern(&mut self, info: TransitInfo) -> TransitInfoId {
        let hash = Self::hash_info(&info);
        if let Some(ids) = self.buckets.get(&hash) {
            for &id in ids {
                if self.infos[id as usize] == info {
                    return id;
                }
            }
        }

        let id = TransitInfoId::try_from(self.infos.len())
            .expect("too many distinct transit lines for u32 IDs");
        self.infos.push(info);
        self.buckets.entry(hash).or_default().push(id);
        id
    }

    pub fn infos(&self) -> &[TransitInfo] {
        &self.infos
    }

    pub fn into_infos(self) -> Vec<TransitInfo> {
        self.infos
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgePL {
    pub geometry: LineString<f64>,
    pub lines: Vec<TransitInfoId>,
    pub oneway: u8,
    pub cost: u32,
    pub level: i32,
    pub restriction: bool,
    pub is_reverse: bool,
    pub allowed_modes: u8,
    pub osmid: i64,
    pub preferred_direction: u8,
}

impl Default for EdgePL {
    fn default() -> Self {
        Self {
            geometry: LineString::new(vec![]),
            lines: Vec::new(),
            oneway: 0,
            cost: 0,
            level: 0,
            restriction: false,
            is_reverse: false,
            allowed_modes: 0,
            osmid: 0,
            preferred_direction: 0,
        }
    }
}

impl EdgePL {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn rev_copy(&self) -> Self {
        let mut ret = self.clone();
        ret.is_reverse = true;
        // logic from C++:
        // if (ret.oneWay() == 1) ret.setOneWay(2);
        // else if (ret.oneWay() == 2) ret.setOneWay(1);
        if ret.oneway == 1 {
            ret.oneway = 2;
        } else if ret.oneway == 2 {
            ret.oneway = 1;
        }
        ret
    }

    pub fn length(&self) -> f64 {
        self.geometry.haversine_length()
    }

    pub fn back_hop(&self) -> Point<f64> {
        let coords = &self.geometry.0;
        if coords.len() < 2 {
            return Point::new(0.0, 0.0);
        }
        let coord = if self.is_reverse {
            coords[1]
        } else {
            coords[coords.len() - 2]
        };
        Point::new(coord.x, coord.y)
    }

    pub fn front_hop(&self) -> Point<f64> {
        let coords = &self.geometry.0;
        if coords.len() < 2 {
            return Point::new(0.0, 0.0);
        }
        let coord = if self.is_reverse {
            coords[coords.len() - 2]
        } else {
            coords[1]
        };
        Point::new(coord.x, coord.y)
    }

    pub fn add_line_id(&mut self, id: TransitInfoId) {
        // C++ EdgePL keeps the shared TransitEdgeLine pointers sorted/unique.
        // Canonical IDs give us the same property without duplicating strings.
        if let Err(idx) = self.lines.binary_search(&id) {
            self.lines.insert(idx, id);
        }
    }

    pub fn get_attrs(&self, transit_lines: &[TransitInfo]) -> AHashMap<String, String> {
        let mut obj = AHashMap::new();
        obj.insert("m_length".to_string(), self.length().to_string());
        obj.insert("oneway".to_string(), self.oneway.to_string());
        obj.insert("cost".to_string(), (self.cost as f64 / 10.0).to_string());
        obj.insert("level".to_string(), self.level.to_string());
        obj.insert(
            "restriction".to_string(),
            if self.restriction { "yes" } else { "no" }.to_string(),
        );
        obj.insert("allowed_modes".to_string(), self.allowed_modes.to_string());
        obj.insert("osmid".to_string(), self.osmid.to_string());
        obj.insert(
            "preferred_direction".to_string(),
            self.preferred_direction.to_string(),
        );

        let lines_str = self
            .lines
            .iter()
            .filter_map(|&id| transit_lines.get(id as usize))
            .map(|l| {
                let mut s = l.short_name.clone();
                if !l.from_str.is_empty() || !l.to_str.is_empty() {
                    s.push_str(&format!("({}->{})", l.from_str, l.to_str));
                }
                s
            })
            .collect::<Vec<_>>()
            .join(",");

        obj.insert("lines".to_string(), lines_str);

        obj
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_edge_pl_defaults() {
        let e = EdgePL::new();
        assert_eq!(e.oneway, 0);
        assert!(!e.restriction);
        assert_eq!(e.cost, 0);
        assert_eq!(e.allowed_modes, 0);
        assert_eq!(e.preferred_direction, 0);
    }

    #[test]
    fn test_rev_copy() {
        let mut e = EdgePL::new();
        e.oneway = 1;

        let rev = e.rev_copy();
        assert!(rev.is_reverse);
        assert_eq!(rev.oneway, 2);
    }

    #[test]
    fn test_transit_lines_are_interned_and_edge_ids_are_unique() {
        let mut interner = TransitInfoInterner::new();
        let a = interner.intern(TransitInfo {
            short_name: "A".into(),
            from_str: "".into(),
            to_str: "".into(),
        });
        let a_again = interner.intern(TransitInfo {
            short_name: "A".into(),
            from_str: "".into(),
            to_str: "".into(),
        });
        let b = interner.intern(TransitInfo {
            short_name: "B".into(),
            from_str: "".into(),
            to_str: "".into(),
        });

        assert_eq!(a, a_again);
        assert_ne!(a, b);
        assert_eq!(interner.infos().len(), 2);

        let mut edge = EdgePL::new();
        edge.add_line_id(b);
        edge.add_line_id(a);
        edge.add_line_id(a_again);
        assert_eq!(edge.lines, vec![a, b]);
    }
}
