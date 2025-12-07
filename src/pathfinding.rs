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
            let neighbor = if edge.from == current {
                edge.to
            } else {
                edge.from
            };

            let tentative_g = current_g + edge.payload.weight;

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
