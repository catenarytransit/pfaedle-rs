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

pub struct TransitMatch {
    pub short_name: Option<String>,
    pub long_name: Option<String>,
    pub operator: Option<String>,
}

pub fn pathfind(
    graph: &Graph<NodePL, EdgePL>,
    start: NodeIndex,
    end: NodeIndex,
    allowed_modes: u8,
    allowed_edges: Option<&AHashSet<usize>>, // EdgeIndices are usize
    preferred_match: Option<&TransitMatch>,
) -> Option<(f64, Vec<usize>)> {
    if start == end {
        return Some((0.0, Vec::new()));
    }

    let end_point = graph.node(end).payload.point;
    let start_point = graph.node(start).payload.point;

    // Heuristics
    let h_fwd = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        p.haversine_distance(&end_point) * 0.1
    };
    let h_bwd = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        p.haversine_distance(&start_point) * 0.1
    };

    let mut open_fwd = BinaryHeap::new();
    let mut open_bwd = BinaryHeap::new();

    let mut came_from_fwd: AHashMap<NodeIndex, (NodeIndex, usize)> = AHashMap::new();
    let mut came_from_bwd: AHashMap<NodeIndex, (NodeIndex, usize)> = AHashMap::new();

    let mut g_fwd: AHashMap<NodeIndex, f64> = AHashMap::new();
    let mut g_bwd: AHashMap<NodeIndex, f64> = AHashMap::new();

    // Initialize
    g_fwd.insert(start, 0.0);
    g_bwd.insert(end, 0.0);

    open_fwd.push(State {
        cost: h_fwd(start),
        node: start,
    });
    open_bwd.push(State {
        cost: h_bwd(end),
        node: end,
    });

    let mut mu = f64::INFINITY;
    let mut meeting_node: Option<(NodeIndex, NodeIndex, usize)> = None; // (u, v, edge_idx)

    // Helper cost calculator
    let get_edge_cost = |edge: &crate::graph::Edge<EdgePL>| -> f64 {
        let mut cost = edge.payload.cost as f64;
        if let Some(pm) = preferred_match {
            for line in &edge.payload.lines {
                let mut matches = false;
                // Check short name
                if let (Some(target), Some(line_name)) = (&pm.short_name, Some(&line.short_name)) {
                    // Heuristic containment
                    if target.contains(&line_name.to_lowercase())
                        || line_name.to_lowercase().contains(target)
                    {
                        matches = true;
                    }
                }
                // Check operator
                if !matches {
                    if let (Some(target_op), Some(line_op)) = (&pm.operator, &line.operator) {
                        if target_op.contains(&line_op.to_lowercase())
                            || line_op.to_lowercase().contains(target_op)
                        {
                            matches = true;
                        }
                    }
                }

                if matches {
                    cost *= 0.2; // Discount for matching lines
                    break;
                }
            }
        }
        cost
    };

    while !open_fwd.is_empty() && !open_bwd.is_empty() {
        // Check termination
        // If the smallest path we could possibly find (min_f + min_b) is worse than best found so far (mu), stop.
        // Note: min_f = g_f + h_f. h_f is distance to end.
        // This standard termination condition requires consistent heuristics.
        let min_f = open_fwd.peek().unwrap().cost;
        let min_b = open_bwd.peek().unwrap().cost;

        // Conservative check: if min_f and min_b are large enough.
        // Since h is scaled by 0.2, it is very admissible.
        if min_f + min_b >= mu {
            // Optimization: Maybe we can stop?
            // With weak heuristic, this might be too loose or tight?
            // Let's rely on queue empty or strict dominance.
            // Actually, with admissible heuristic, this condition (min_f + min_b >= mu) is valid for optimal path.
            // But we must be careful about heuristic consistency across directions.
            // Prudence: use a slightly looser bound or just let it run until exhaustion or clear dominance?
            // Standard Bi-A* with consistent heuristic stops here.
            break;
        }

        // Expand the direction with smaller frontier size to keep balance
        let expand_fwd = open_fwd.len() <= open_bwd.len();

        if expand_fwd {
            if let Some(State { cost: _, node: u }) = open_fwd.pop() {
                let current_g = *g_fwd.get(&u).unwrap_or(&f64::INFINITY);

                // Pruning if we already found a path better than this node's theoretical best
                // But we modify mu dynamically.
                if current_g + h_fwd(u) >= mu {
                    continue;
                }

                for &edge_idx in &graph.node(u).edges {
                    if let Some(allowed) = allowed_edges {
                        if !allowed.contains(&edge_idx) {
                            continue;
                        }
                    }
                    let edge = graph.edge(edge_idx);
                    if (edge.payload.allowed_modes & allowed_modes) == 0 {
                        continue;
                    }

                    let v = if edge.from == u { edge.to } else { edge.from };
                    let cost = get_edge_cost(edge);
                    let tentative_g = current_g + cost;

                    if tentative_g < *g_fwd.get(&v).unwrap_or(&f64::INFINITY) {
                        g_fwd.insert(v, tentative_g);
                        came_from_fwd.insert(v, (u, edge_idx));
                        open_fwd.push(State {
                            cost: tentative_g + h_fwd(v),
                            node: v,
                        });

                        // Check intersection with backward search
                        if let Some(&g_b) = g_bwd.get(&v) {
                            let dist = tentative_g + g_b; // We check dist through v? No, this is meeting at V.
                            // But wait, my logic was meeting at EDGE.
                            // If `v` is in `g_bwd`, it means we have a path `end -> ... -> v` with cost `g_b`.
                            // So total cost is `tentative_g` (start->u->v) + `g_b` (v->end).
                            // This corresponds to meeting at node `v`.
                            // Wait, if meeting at node `v`, which edge is the "meeting edge"?
                            // It's the edge `u->v` (edge_idx) we just traversed.
                            if dist < mu {
                                mu = dist;
                                meeting_node = Some((u, v, edge_idx));
                            }
                        }
                    }
                }
            }
        } else {
            if let Some(State { cost: _, node: u }) = open_bwd.pop() {
                // u is current in bwd search
                let current_g = *g_bwd.get(&u).unwrap_or(&f64::INFINITY);

                if current_g + h_bwd(u) >= mu {
                    continue;
                }

                for &edge_idx in &graph.node(u).edges {
                    if let Some(allowed) = allowed_edges {
                        if !allowed.contains(&edge_idx) {
                            continue;
                        }
                    }
                    let edge = graph.edge(edge_idx);
                    if (edge.payload.allowed_modes & allowed_modes) == 0 {
                        continue;
                    }

                    let v = if edge.from == u { edge.to } else { edge.from };
                    let cost = get_edge_cost(edge);
                    let tentative_g = current_g + cost;

                    if tentative_g < *g_bwd.get(&v).unwrap_or(&f64::INFINITY) {
                        g_bwd.insert(v, tentative_g);
                        came_from_bwd.insert(v, (u, edge_idx));
                        open_bwd.push(State {
                            cost: tentative_g + h_bwd(v),
                            node: v,
                        });

                        // Check intersection with forward search
                        if let Some(&g_f) = g_fwd.get(&v) {
                            // Path: start -> ... -> v (cost g_f) -> u -> ... -> end (cost tentative_g via edge)
                            // Total: g_f + tentative_g
                            let dist = g_f + tentative_g;
                            if dist < mu {
                                mu = dist;
                                // Meeting at node v? No, traversing v->u (backward) means u->v (forward).
                                // Edge is `edge_idx` connecting u and v.
                                // We are coming from u (bwd) to v (bwd neighbor).
                                // So path is start->...->v + edge(v,u) + u->...->end.
                                // Edge connects v and u.
                                // record as (v, u, edge_idx)?
                                // My reconstruction expects (u_fwd, v_bwd, edge).
                                // Here `v` is in fwd tree. `u` is in bwd tree.
                                // So (v, u, edge_idx).
                                meeting_node = Some((v, u, edge_idx));
                            }
                        }
                    }
                }
            }
        }
    }

    if let Some((u, v, mid_edge)) = meeting_node {
        // Reconstruct path
        // start -> ... -> u
        let mut path = reconstruct_path(came_from_fwd, u);

        // Relationship check: mid_edge connects u and v.
        // path contains edges leading up to u.
        // We push mid_edge (u -> v).
        path.push(mid_edge);

        // v -> ... -> end
        // reconstruct_path(came_from_bwd, v) gives [e_near_end, ..., e_near_v].
        // These are edges leading to v from end.
        // We want edges from v to end.
        let mut path_bwd = reconstruct_path(came_from_bwd, v);
        path_bwd.reverse();
        path.extend(path_bwd);

        return Some((mu, path));
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
        let path_rail = pathfind(&graph, n0, n1, MODE_RAIL, None, None);
        assert!(path_rail.is_some());
        let (_, edges) = path_rail.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_RAIL);

        // Test Bus
        let path_bus = pathfind(&graph, n0, n1, MODE_BUS, None, None);
        assert!(path_bus.is_some());
        let (_, edges) = path_bus.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_BUS);

        // Test None
        let path_none = pathfind(&graph, n0, n1, 0, None, None);
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
        let result = pathfind(&graph, n0, n1, MODE_RAIL, None, None).expect("Should find path");
        let (_, path) = result;

        // Should choose the detour (2 edges) because total cost 2000 < 1,000,000
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], idx_norm1);
        assert_eq!(path[1], idx_norm2);

        // Ensure it didn't take the direct industrial route
        assert!(!path.contains(&idx_ind));
    }

    #[test]
    fn test_pathfind_preference() {
        let mut graph = Graph::new();
        let n0 = graph.add_node(NodePL {
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            point: Point::new(0.0, 0.1),
        });

        use crate::graph::TransitInfo;

        // Edge 0: Direct but wrong operator
        // Cost = 12000 (approx 11km)
        let mut e_wrong = EdgePL::new();
        e_wrong.allowed_modes = MODE_RAIL;
        e_wrong.cost = 12000;
        e_wrong.add_line(TransitInfo {
            short_name: "L1".into(),
            from_str: "".into(),
            to_str: "".into(),
            operator: Some("OtherOp".into()),
        });
        let idx_wrong = graph.add_edge(n0, n1, e_wrong);

        // Edge 1: Detour but correct operator
        // Cost = 7000 * 2 = 14000 (longer)
        // With preference ("MyOp"), cost becomes 14000 * 0.2 = 2800.
        let n2 = graph.add_node(NodePL {
            point: Point::new(0.1, 0.05),
        });

        let mut e_right1 = EdgePL::new();
        e_right1.allowed_modes = MODE_RAIL;
        e_right1.cost = 7000;
        e_right1.add_line(TransitInfo {
            short_name: "L2".into(),
            from_str: "".into(),
            to_str: "".into(),
            operator: Some("MyOp".into()),
        });
        let idx_right1 = graph.add_edge(n0, n2, e_right1);

        let mut e_right2 = EdgePL::new();
        e_right2.allowed_modes = MODE_RAIL;
        e_right2.cost = 7000;
        e_right2.add_line(TransitInfo {
            short_name: "L2".into(),
            from_str: "".into(),
            to_str: "".into(),
            operator: Some("MyOp".into()),
        });
        let idx_right2 = graph.add_edge(n2, n1, e_right2);

        let match_pref = TransitMatch {
            short_name: None,
            long_name: None,
            operator: Some("myop".to_string()), // Lowercase for matching
        };

        // Pathfind with preference
        let result =
            pathfind(&graph, n0, n1, MODE_RAIL, None, Some(&match_pref)).expect("Should find path");
        let (_, path) = result;

        // Should choose path via n2 because (55+55)*0.2 = 22 < 100
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], idx_right1);
        assert_eq!(path[1], idx_right2);

        assert!(!path.contains(&idx_wrong));
    }
}
