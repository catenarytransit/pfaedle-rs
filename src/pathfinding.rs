use crate::graph::{EdgePL, Graph, NodeIndex, NodePL};
use geo::algorithm::VincentyDistance;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: f64,
    node: NodeIndex,
}

// Rust's BinaryHeap is max-heap, so we flip ordering to get min-heap
impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // partial_cmp reverse for min-heap
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn pathfind(
    graph: &Graph<NodePL, EdgePL>,
    start: NodeIndex,
    end: NodeIndex,
    allowed_modes: u8,
) -> Option<Vec<usize>> {
    // Returns list of EdgeIndices
    let end_point = graph.node(end).payload.point;

    let heuristic = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        p.vincenty_distance(&end_point).unwrap_or(0.0)
    };

    let mut open_set = BinaryHeap::new();
    open_set.push(State {
        cost: 0.0,
        node: start,
    });

    let mut came_from: HashMap<NodeIndex, (NodeIndex, usize)> = HashMap::new(); // Node -> (ParentNode, EdgeIndex)
    let mut g_score: HashMap<NodeIndex, f64> = HashMap::new();

    g_score.insert(start, 0.0);

    while let Some(State {
        cost: _,
        node: current,
    }) = open_set.pop()
    {
        if current == end {
            return Some(reconstruct_path(came_from, current));
        }

        let current_g = *g_score.get(&current).unwrap_or(&f64::INFINITY);

        for &edge_idx in &graph.node(current).edges {
            let edge = graph.edge(edge_idx);

            // Filter by mode
            if (edge.payload.allowed_modes & allowed_modes) == 0 {
                continue;
            }

            let neighbor = if edge.from == current {
                edge.to
            } else {
                edge.from
            };

            let tentative_g = current_g + edge.payload.length();

            if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                came_from.insert(neighbor, (current, edge_idx));
                g_score.insert(neighbor, tentative_g);
                let f_score = tentative_g + heuristic(neighbor);
                open_set.push(State {
                    cost: f_score,
                    node: neighbor,
                });
            }
        }
    }

    None
}

fn reconstruct_path(
    came_from: HashMap<NodeIndex, (NodeIndex, usize)>,
    mut current: NodeIndex,
) -> Vec<usize> {
    let mut path = Vec::new();
    while let Some(&(parent, edge_idx)) = came_from.get(&current) {
        path.push(edge_idx);
        current = parent;
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{EdgePL, Graph, MODE_BUS, MODE_RAIL, NodePL};
    use geo::Point;

    #[test]
    fn test_pathfind_modes() {
        let mut graph = Graph::new();
        let n0 = graph.add_node(NodePL {
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            point: Point::new(1.0, 0.0), // Far enough to have cost
        });

        // Edge 0: Rail only
        let mut e0 = EdgePL::new();
        e0.allowed_modes = MODE_RAIL;
        e0.cost = 10;
        graph.add_edge(n0, n1, e0);

        // Edge 1: Bus only (higher cost just to distinguish if needed, but we filter)
        let mut e1 = EdgePL::new();
        e1.allowed_modes = MODE_BUS;
        e1.cost = 10;
        graph.add_edge(n0, n1, e1);

        // Test Rail
        let path_rail = pathfind(&graph, n0, n1, MODE_RAIL);
        assert!(path_rail.is_some());
        let edges = path_rail.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_RAIL);

        // Test Bus
        let path_bus = pathfind(&graph, n0, n1, MODE_BUS);
        assert!(path_bus.is_some());
        let edges = path_bus.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_BUS);

        // Test None
        let path_none = pathfind(&graph, n0, n1, 0);
        assert!(path_none.is_none());
    }
}
