use crate::graph::{EdgePL, Graph, NodeIndex, NodePL};
use ahash::{AHashMap, AHashSet};
use geo::algorithm::HaversineDistance;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

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
    allowed_edges: Option<&AHashSet<usize>>, // EdgeIndices are usize
) -> Option<(f64, Vec<usize>)> {
    // Returns (TotalCost, list of EdgeIndices)
    let end_point = graph.node(end).payload.point;

    // Heuristic needs to be admissible (<= actual cost).
    // Cost is roughly time_seconds * 10
    // Max speed ~ 100km/h ~ 27m/s.
    // Min cost per meter = (1 / 27) * 10 ~ 0.37
    // Let's use 0.1 * distance to be safe and simple.
    let heuristic = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        p.haversine_distance(&end_point) * 0.1
    };

    let mut open_set = BinaryHeap::new();
    open_set.push(State {
        cost: 0.0,
        node: start,
    });

    let mut came_from: AHashMap<NodeIndex, (NodeIndex, usize)> = AHashMap::new(); // Node -> (ParentNode, EdgeIndex)
    let mut g_score: AHashMap<NodeIndex, f64> = AHashMap::new();

    g_score.insert(start, 0.0);

    while let Some(State {
        cost: _,
        node: current,
    }) = open_set.pop()
    {
        if current == end {
            let total_cost = *g_score.get(&current).unwrap();
            return Some((total_cost, reconstruct_path(came_from, current)));
        }

        let current_g = *g_score.get(&current).unwrap_or(&f64::INFINITY);

        for &edge_idx in &graph.node(current).edges {
            // Check specific allowed edges if provided
            if let Some(allowed) = allowed_edges {
                if !allowed.contains(&edge_idx) {
                    continue;
                }
            }

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

            // Use logical cost instead of physical length
            let tentative_g = current_g + edge.payload.cost as f64;

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
    came_from: AHashMap<NodeIndex, (NodeIndex, usize)>,
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
        let path_rail = pathfind(&graph, n0, n1, MODE_RAIL, None);
        assert!(path_rail.is_some());
        let (_, edges) = path_rail.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_RAIL);

        // Test Bus
        let path_bus = pathfind(&graph, n0, n1, MODE_BUS, None);
        assert!(path_bus.is_some());
        let (_, edges) = path_bus.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_BUS);

        // Test None
        let path_none = pathfind(&graph, n0, n1, 0, None);
        assert!(path_none.is_none());
    }

    #[test]
    fn test_pathfind_penalties() {
        let mut graph = Graph::new();
        let n0 = graph.add_node(NodePL {
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            point: Point::new(0.0, 0.1),
        });

        // Edge 0: Short but HIGH COST (e.g. industrial)
        // Length ~ 11km.
        let mut e_industrial = EdgePL::new();
        e_industrial.allowed_modes = MODE_RAIL;
        e_industrial.cost = 1_000_000;
        let idx_ind = graph.add_edge(n0, n1, e_industrial);

        // Edge 1: Long but LOW COST (normal rail)
        // We'll simulate a detour via n2
        let n2 = graph.add_node(NodePL {
            point: Point::new(0.1, 0.05),
        });

        let mut e_normal1 = EdgePL::new();
        e_normal1.allowed_modes = MODE_RAIL;
        e_normal1.cost = 1000; // Low cost
        let idx_norm1 = graph.add_edge(n0, n2, e_normal1);

        let mut e_normal2 = EdgePL::new();
        e_normal2.allowed_modes = MODE_RAIL;
        e_normal2.cost = 1000; // Low cost
        let idx_norm2 = graph.add_edge(n2, n1, e_normal2);

        // Pathfind
        let result = pathfind(&graph, n0, n1, MODE_RAIL, None).expect("Should find path");
        let (_, path) = result;

        // Should choose the detour (2 edges) because total cost 2000 < 1,000,000
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], idx_norm1);
        assert_eq!(path[1], idx_norm2);

        // Ensure it didn't take the direct industrial route
        assert!(!path.contains(&idx_ind));
    }
}
