use geo::{LineString, Point};

pub type NodeIndex = usize;
pub type EdgeIndex = usize;

#[derive(Debug, Clone)]
pub struct Node<N> {
    pub payload: N,
    pub edges: Vec<EdgeIndex>,
}

#[derive(Debug, Clone)]
pub struct Edge<E> {
    pub payload: E,
    pub from: NodeIndex,
    pub to: NodeIndex,
}

#[derive(Debug, Clone)]
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
            edges: Vec::new(),
        });
        index
    }

    pub fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, payload: E) -> EdgeIndex {
        let index = self.edges.len();
        self.edges.push(Edge { payload, from, to });

        self.nodes[from].edges.push(index);
        // Undirected graph logic: add to both unless loop
        if from != to {
            self.nodes[to].edges.push(index);
        }

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

#[derive(Debug, Clone)]
pub struct NodePL {
    pub point: Point<f64>,
}

#[derive(Debug, Clone)]
pub struct EdgePL {
    pub geometry: LineString<f64>,
    pub weight: f64, // Length in meters
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_graph_construction() {
        let mut graph: Graph<NodePL, EdgePL> = Graph::new();
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(1.0, 1.0);

        let n1 = graph.add_node(NodePL { point: p1 });
        let n2 = graph.add_node(NodePL { point: p2 });

        // Add edge
        let geom = LineString::new(vec![p1.into(), p2.into()]);
        let e1 = graph.add_edge(
            n1,
            n2,
            EdgePL {
                geometry: geom,
                weight: 10.0,
            },
        );

        assert_eq!(graph.nodes.len(), 2);
        assert_eq!(graph.edges.len(), 1);

        // Check adjacency
        assert_eq!(graph.node(n1).edges.len(), 1);
        assert_eq!(graph.node(n2).edges.len(), 1);
        assert_eq!(graph.node(n1).edges[0], e1);
    }
}
