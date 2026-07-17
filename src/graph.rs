use ahash::AHashMap;
use geo::{HaversineLength, LineString, Point};
use serde::{Deserialize, Serialize};

pub type NodeIndex = usize;
pub type EdgeIndex = usize;

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
}

impl<N, E> Graph<N, E> {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
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
}

// Payloads corresponding to pfaedle's netgraph

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodePL {
    pub point: Point<f64>,
    #[serde(default)]
    pub comp_id: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct TransitInfo {
    pub short_name: String,
    pub from_str: String,
    pub to_str: String,
    pub operator: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgePL {
    pub geometry: LineString<f64>,
    pub lines: Vec<TransitInfo>,
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

    pub fn add_line(&mut self, info: TransitInfo) {
        // C++:
        // auto lb = std::lower_bound(_lines.begin(), _lines.end(), l);
        // if (lb == _lines.end() || *lb != l) { _lines.insert(lb, l); ... }
        // Maintains sorted order and uniqueness.

        let pos = self.lines.binary_search(&info);
        if let Err(idx) = pos {
            self.lines.insert(idx, info);
        }
    }

    pub fn get_attrs(&self) -> AHashMap<String, String> {
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
            .map(|l| {
                let mut s = l.short_name.clone();
                if !l.from_str.is_empty() || !l.to_str.is_empty() {
                    s.push_str(&format!("({}->{})", l.from_str, l.to_str));
                }
                if let Some(op) = &l.operator {
                    s.push_str(&format!(" [{}]", op));
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
    fn test_transit_lines_sorted() {
        let mut e = EdgePL::new();
        let t1 = TransitInfo {
            short_name: "B".into(),
            from_str: "".into(),
            to_str: "".into(),
            operator: None,
        };
        let t2 = TransitInfo {
            short_name: "A".into(),
            from_str: "".into(),
            to_str: "".into(),
            operator: Some("OpA".into()),
        };

        e.add_line(t1.clone());
        e.add_line(t2.clone());
        e.add_line(t1.clone()); // Duplicate

        assert_eq!(e.lines.len(), 2);
        assert_eq!(e.lines[0], t2);
        assert_eq!(e.lines[1], t1);
    }
}
