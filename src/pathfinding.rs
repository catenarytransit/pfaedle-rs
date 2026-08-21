use crate::graph::{EdgePL, Graph, NodeIndex, NodePL};
use ahash::{AHashMap, AHashSet};
use geo::Point;

use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: f64,
    g: f64,
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

fn heuristic_m(p: Point<f64>, end_lat: f64, end_lon: f64, cos_end_lat: f64) -> f64 {
    // Point: x=lon, y=lat
    let dy = (p.y() - end_lat) * 111_320.0;
    let dx = (p.x() - end_lon) * 111_320.0 * cos_end_lat;
    (dx * dx + dy * dy).sqrt()
}

#[derive(Clone)]
pub struct PathfinderContext {
    open_fwd: BinaryHeap<State>,
    open_bwd: BinaryHeap<State>,
    came_from_fwd: AHashMap<NodeIndex, (NodeIndex, usize)>,
    came_from_bwd: AHashMap<NodeIndex, (NodeIndex, usize)>,
    g_fwd: AHashMap<NodeIndex, f64>,
    g_bwd: AHashMap<NodeIndex, f64>,
}

impl PathfinderContext {
    pub fn new() -> Self {
        Self {
            open_fwd: BinaryHeap::new(),
            open_bwd: BinaryHeap::new(),
            came_from_fwd: AHashMap::new(),
            came_from_bwd: AHashMap::new(),
            g_fwd: AHashMap::new(),
            g_bwd: AHashMap::new(),
        }
    }

    pub fn reset(&mut self) {
        self.open_fwd.clear();
        self.open_bwd.clear();
        self.came_from_fwd.clear();
        self.came_from_bwd.clear();
        self.g_fwd.clear();
        self.g_bwd.clear();
    }

    pub fn retained_entries(&self) -> usize {
        self.open_fwd.capacity()
            + self.open_bwd.capacity()
            + self.came_from_fwd.capacity()
            + self.came_from_bwd.capacity()
            + self.g_fwd.capacity()
            + self.g_bwd.capacity()
    }

    pub fn discard_if_oversized(&mut self, maximum_entries: usize) {
        if self.retained_entries() > maximum_entries {
            *self = Self::new();
        }
    }
}

pub fn pathfind(
    graph: &Graph<NodePL, EdgePL>,
    start: NodeIndex,
    end: NodeIndex,
    allowed_modes: u8,
    fallback_modes: u8,
    allowed_edges: Option<&AHashSet<usize>>,
    preferred_match: Option<&TransitMatch>,
) -> Option<(f64, Vec<usize>)> {
    let mut ctx = PathfinderContext::new();
    pathfind_with_context(
        &mut ctx,
        graph,
        start,
        end,
        allowed_modes,
        fallback_modes,
        allowed_edges,
        preferred_match,
        None,
    )
}

fn contains_ignore_case(corpus: &str, target_lower: &str) -> bool {
    // Check if `corpus` (mixed case) contains `target_lower` (lowercase)
    // without allocating a new lowercase string for `corpus`.
    if target_lower.is_empty() {
        return true;
    }
    if corpus.len() < target_lower.len() {
        return false;
    }

    let target_bytes = target_lower.as_bytes();
    let corpus_bytes = corpus.as_bytes();

    // Naive search is O(N*M), but strings are short (transit lines).
    // Boyer-Moore or similar is overkill here.
    // We iterate through valid start positions
    for i in 0..=(corpus.len() - target_lower.len()) {
        let mut match_found = true;
        for j in 0..target_lower.len() {
            let c = corpus_bytes[i + j];
            let shift = if c >= b'A' && c <= b'Z' {
                c + 32 // to lower
            } else {
                c
            };
            if shift != target_bytes[j] {
                match_found = false;
                break;
            }
        }
        if match_found {
            return true;
        }
    }
    false
}

pub fn pathfind_with_context(
    ctx: &mut PathfinderContext,
    graph: &Graph<NodePL, EdgePL>,
    start: NodeIndex,
    end: NodeIndex,
    allowed_modes: u8,
    fallback_modes: u8,
    allowed_edges: Option<&AHashSet<usize>>, // EdgeIndices are usize
    preferred_match: Option<&TransitMatch>,
    bounding_box: Option<(f64, f64, f64, f64)>, // (min_lon, min_lat, max_lon, max_lat)
) -> Option<(f64, Vec<usize>)> {
    ctx.reset();

    if start == end {
        return Some((0.0, Vec::new()));
    }

    let end_point = graph.node(end).payload.point;
    let start_point = graph.node(start).payload.point;

    // Heuristics
    let end_lat = end_point.y();
    let end_lon = end_point.x();
    let end_cos = end_lat.to_radians().cos();

    let start_lat = start_point.y();
    let start_lon = start_point.x();
    let start_cos = start_lat.to_radians().cos();

    // Heuristics
    let heuristic_factor = if allowed_modes == crate::graph::MODE_BUS {
        0.7
    } else {
        0.1
    };

    let h_fwd = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        heuristic_m(p, end_lat, end_lon, end_cos) * heuristic_factor
    };
    let h_bwd = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        heuristic_m(p, start_lat, start_lon, start_cos) * heuristic_factor
    };

    // Initialize
    ctx.g_fwd.insert(start, 0.0);
    ctx.g_bwd.insert(end, 0.0);

    ctx.open_fwd.push(State {
        cost: h_fwd(start),
        g: 0.0,
        node: start,
    });
    ctx.open_bwd.push(State {
        cost: h_bwd(end),
        g: 0.0,
        node: end,
    });

    let mut mu = f64::INFINITY;
    let mut meeting_node: Option<(NodeIndex, NodeIndex, usize)> = None; // (u, v, edge_idx)

    // Helper cost calculator
    let get_edge_cost = |edge: &crate::graph::Edge<EdgePL>| -> f64 {
        let mut cost = edge.payload.cost as f64;
        if let Some(pm) = preferred_match {
            for &line_id in &edge.payload.lines {
                let Some(line) = graph.transit_info(line_id) else {
                    continue;
                };
                let mut matches = false;
                // Check short name
                if let (Some(target), Some(line_name)) = (&pm.short_name, Some(&line.short_name)) {
                    // Optimized containment check
                    // We assume `target` is already lowercase (from matcher.rs)
                    if contains_ignore_case(line_name, target)
                        || target.contains(&line_name.to_lowercase())
                    // Keeping reverse check (if target is "Bus 100" and line is "100")
                    {
                        matches = true;
                    }
                }
                // Check long name
                if !matches {
                    if let Some(target) = &pm.long_name {
                        let line_name = &line.short_name;
                        if contains_ignore_case(line_name, target)
                            || target.contains(&line_name.to_lowercase())
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

    while !ctx.open_fwd.is_empty() && !ctx.open_bwd.is_empty() {
        // Check termination
        let min_f = ctx.open_fwd.peek().unwrap().cost;
        let min_b = ctx.open_bwd.peek().unwrap().cost;

        if min_f + min_b >= mu {
            break;
        }

        // Expand the direction with smaller frontier size to keep balance
        let expand_fwd = ctx.open_fwd.len() <= ctx.open_bwd.len();

        if expand_fwd {
            if let Some(State {
                cost: _,
                g,
                node: u,
            }) = ctx.open_fwd.pop()
            {
                let current_g = *ctx.g_fwd.get(&u).unwrap_or(&f64::INFINITY);

                if g > current_g || current_g + h_fwd(u) >= mu {
                    continue;
                }

                for &edge_idx in &graph.node(u).out_edges {
                    if let Some(allowed) = allowed_edges {
                        if !allowed.contains(&edge_idx) {
                            continue;
                        }
                    }
                    let edge = graph.edge(edge_idx);
                    if (edge.payload.allowed_modes & (allowed_modes | fallback_modes)) == 0 {
                        continue;
                    }

                    let v = if edge.from == u { edge.to } else { edge.from };
                    let mut cost = get_edge_cost(edge);
                    if (edge.payload.allowed_modes & allowed_modes) == 0 {
                        cost *= 10.0;
                    }
                    let tentative_g = current_g + cost;

                    if tentative_g < *ctx.g_fwd.get(&v).unwrap_or(&f64::INFINITY) {
                        ctx.g_fwd.insert(v, tentative_g);
                        ctx.came_from_fwd.insert(v, (u, edge_idx));
                        ctx.open_fwd.push(State {
                            cost: tentative_g + h_fwd(v),
                            g: tentative_g,
                            node: v,
                        });

                        // Check intersection with backward search
                        if let Some(&g_b) = ctx.g_bwd.get(&v) {
                            let dist = tentative_g + g_b;
                            if dist < mu {
                                mu = dist;
                                meeting_node = Some((u, v, edge_idx));
                            }
                        }
                    }
                }
            }
        } else {
            if let Some(State {
                cost: _,
                g,
                node: u,
            }) = ctx.open_bwd.pop()
            {
                let current_g = *ctx.g_bwd.get(&u).unwrap_or(&f64::INFINITY);

                if g > current_g || current_g + h_bwd(u) >= mu {
                    continue;
                }

                for &edge_idx in &graph.node(u).in_edges {
                    if let Some(allowed) = allowed_edges {
                        if !allowed.contains(&edge_idx) {
                            continue;
                        }
                    }
                    let edge = graph.edge(edge_idx);
                    if (edge.payload.allowed_modes & (allowed_modes | fallback_modes)) == 0 {
                        continue;
                    }

                    let v = if edge.from == u { edge.to } else { edge.from };
                    let mut cost = get_edge_cost(edge);
                    if (edge.payload.allowed_modes & allowed_modes) == 0 {
                        cost *= 10.0;
                    }
                    let tentative_g = current_g + cost;

                    if tentative_g < *ctx.g_bwd.get(&v).unwrap_or(&f64::INFINITY) {
                        ctx.g_bwd.insert(v, tentative_g);
                        ctx.came_from_bwd.insert(v, (u, edge_idx));
                        ctx.open_bwd.push(State {
                            cost: tentative_g + h_bwd(v),
                            g: tentative_g,
                            node: v,
                        });

                        // Check intersection with forward search
                        if let Some(&g_f) = ctx.g_fwd.get(&v) {
                            let dist = g_f + tentative_g;
                            if dist < mu {
                                mu = dist;
                                meeting_node = Some((v, u, edge_idx));
                            }
                        }
                    }
                }
            }
        }
    }

    if let Some((u, v, mid_edge)) = meeting_node {
        let mut path = reconstruct_path(&ctx.came_from_fwd, u);
        path.push(mid_edge);
        let mut path_bwd = reconstruct_path(&ctx.came_from_bwd, v);
        path_bwd.reverse();
        path.extend(path_bwd);

        return Some((mu, path));
    }

    None
}

fn reconstruct_path(
    came_from: &AHashMap<NodeIndex, (NodeIndex, usize)>,
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
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
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
        let path_rail = pathfind(&graph, n0, n1, MODE_RAIL, 0, None, None);
        assert!(path_rail.is_some());
        let (_, edges) = path_rail.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_RAIL);

        // Test Bus
        let path_bus = pathfind(&graph, n0, n1, MODE_BUS, 0, None, None);
        assert!(path_bus.is_some());
        let (_, edges) = path_bus.unwrap();
        assert_eq!(edges.len(), 1);
        assert_eq!(graph.edge(edges[0]).payload.allowed_modes, MODE_BUS);

        // Test None
        let path_none = pathfind(&graph, n0, n1, 0, 0, None, None);
        assert!(path_none.is_none());
    }

    #[test]
    fn test_pathfind_penalties() {
        let mut graph = Graph::new();
        let n0 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
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
            comp_id: 0,
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
        let result = pathfind(&graph, n0, n1, MODE_RAIL, 0, None, None).expect("Should find path");
        let (_, path) = result;

        // Should choose the detour (2 edges) because total cost 2000 < 1,000,000
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], idx_norm1);
        assert_eq!(path[1], idx_norm2);

        // Ensure it didn't take the direct industrial route
        assert!(!path.contains(&idx_ind));
    }

    #[test]
    fn test_pathfind_preference_by_short_name() {
        let mut graph = Graph::new();
        graph.transit_lines = vec![
            crate::graph::TransitInfo {
                short_name: "L1".into(),
                from_str: String::new(),
                to_str: String::new(),
            },
            crate::graph::TransitInfo {
                short_name: "L2".into(),
                from_str: String::new(),
                to_str: String::new(),
            },
        ];
        let n0 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.1),
        });
        let n2 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.1, 0.05),
        });

        let mut e_wrong = EdgePL::new();
        e_wrong.allowed_modes = MODE_RAIL;
        e_wrong.cost = 12000;
        e_wrong.lines = vec![0];
        let idx_wrong = graph.add_edge(n0, n1, e_wrong);

        let mut e_right1 = EdgePL::new();
        e_right1.allowed_modes = MODE_RAIL;
        e_right1.cost = 7000;
        e_right1.lines = vec![1];
        let idx_right1 = graph.add_edge(n0, n2, e_right1);

        let mut e_right2 = EdgePL::new();
        e_right2.allowed_modes = MODE_RAIL;
        e_right2.cost = 7000;
        e_right2.lines = vec![1];
        let idx_right2 = graph.add_edge(n2, n1, e_right2);

        let match_pref = TransitMatch {
            short_name: Some("l2".to_string()),
            long_name: None,
            operator: None,
        };

        let (_, path) = pathfind(&graph, n0, n1, MODE_RAIL, 0, None, Some(&match_pref))
            .expect("Should find path");
        assert_eq!(path, vec![idx_right1, idx_right2]);
        assert!(!path.contains(&idx_wrong));
    }

    #[test]
    fn test_pathfind_preference_by_long_name() {
        let mut graph = Graph::new();
        graph.transit_lines = vec![
            crate::graph::TransitInfo {
                short_name: "Blue".into(),
                from_str: String::new(),
                to_str: String::new(),
            },
            crate::graph::TransitInfo {
                short_name: "Red".into(),
                from_str: String::new(),
                to_str: String::new(),
            },
        ];
        let n0 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.0),
        });
        let n1 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.0, 0.1),
        });
        let n2 = graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(0.1, 0.05),
        });

        let mut e_other = EdgePL::new();
        e_other.allowed_modes = MODE_RAIL;
        e_other.cost = 12000;
        e_other.lines = vec![0];
        let idx_other = graph.add_edge(n0, n1, e_other);

        let mut e_red1 = EdgePL::new();
        e_red1.allowed_modes = MODE_RAIL;
        e_red1.cost = 7000;
        e_red1.lines = vec![1];
        let idx_red1 = graph.add_edge(n0, n2, e_red1);

        let mut e_red2 = EdgePL::new();
        e_red2.allowed_modes = MODE_RAIL;
        e_red2.cost = 7000;
        e_red2.lines = vec![1];
        let idx_red2 = graph.add_edge(n2, n1, e_red2);

        let match_pref = TransitMatch {
            short_name: None,
            long_name: Some("red line".to_string()),
            operator: None,
        };

        let (_, path) = pathfind(&graph, n0, n1, MODE_RAIL, 0, None, Some(&match_pref))
            .expect("Should find path");
        assert_eq!(path, vec![idx_red1, idx_red2]);
        assert!(!path.contains(&idx_other));

        let mut ctx = PathfinderContext::new();
        let cost = pathfind_cost_with_context(
            &mut ctx,
            &graph,
            n0,
            n1,
            MODE_RAIL,
            0,
            None,
            Some(&match_pref),
            None,
        )
        .expect("Should find path cost");
        assert_eq!(cost, 2800.0);
    }
}

pub fn pathfind_cost_with_context(
    ctx: &mut PathfinderContext,
    graph: &Graph<NodePL, EdgePL>,
    start: NodeIndex,
    end: NodeIndex,
    allowed_modes: u8,
    fallback_modes: u8,
    allowed_edges: Option<&AHashSet<usize>>, // EdgeIndices are usize
    preferred_match: Option<&TransitMatch>,
    bounding_box: Option<(f64, f64, f64, f64)>, // (min_lon, min_lat, max_lon, max_lat)
) -> Option<f64> {
    ctx.reset();

    if start == end {
        return Some(0.0);
    }

    let end_point = graph.node(end).payload.point;
    let start_point = graph.node(start).payload.point;

    // Heuristics
    let end_lat = end_point.y();
    let end_lon = end_point.x();
    let end_cos = end_lat.to_radians().cos();

    let start_lat = start_point.y();
    let start_lon = start_point.x();
    let start_cos = start_lat.to_radians().cos();

    // Heuristics
    let heuristic_factor = if allowed_modes == crate::graph::MODE_BUS {
        0.7
    } else {
        0.1
    };

    let h_fwd = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        heuristic_m(p, end_lat, end_lon, end_cos) * heuristic_factor
    };
    let h_bwd = |n: NodeIndex| -> f64 {
        let p = graph.node(n).payload.point;
        heuristic_m(p, start_lat, start_lon, start_cos) * heuristic_factor
    };

    // Initialize
    ctx.g_fwd.insert(start, 0.0);
    ctx.g_bwd.insert(end, 0.0);

    ctx.open_fwd.push(State {
        cost: h_fwd(start),
        g: 0.0,
        node: start,
    });
    ctx.open_bwd.push(State {
        cost: h_bwd(end),
        g: 0.0,
        node: end,
    });

    let mut mu = f64::INFINITY;

    // Helper cost calculator
    let get_edge_cost = |edge: &crate::graph::Edge<EdgePL>| -> f64 {
        let mut cost = edge.payload.cost as f64;
        if let Some(pm) = preferred_match {
            for &line_id in &edge.payload.lines {
                let Some(line) = graph.transit_info(line_id) else {
                    continue;
                };
                let mut matches = false;
                if let Some(target) = &pm.short_name {
                    let line_name = &line.short_name;
                    if contains_ignore_case(line_name, target)
                        || target.contains(&line_name.to_lowercase())
                    {
                        matches = true;
                    }
                }
                // The C++ TransitEdgeLine identity is short-name/from/to only.
                // Rust also accepts the GTFS long name as an alternate query
                // against the OSM relation line name, but never operator here.
                if !matches {
                    if let Some(target) = &pm.long_name {
                        let line_name = &line.short_name;
                        if contains_ignore_case(line_name, target)
                            || target.contains(&line_name.to_lowercase())
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

    while !ctx.open_fwd.is_empty() && !ctx.open_bwd.is_empty() {
        let min_f = ctx.open_fwd.peek().unwrap().cost;
        let min_b = ctx.open_bwd.peek().unwrap().cost;

        if min_f + min_b >= mu {
            break;
        }

        let expand_fwd = ctx.open_fwd.len() <= ctx.open_bwd.len();

        if expand_fwd {
            if let Some(State {
                cost: _,
                g,
                node: u,
            }) = ctx.open_fwd.pop()
            {
                let current_g = *ctx.g_fwd.get(&u).unwrap_or(&f64::INFINITY);

                if g > current_g || current_g + h_fwd(u) >= mu {
                    continue;
                }

                for &edge_idx in &graph.node(u).out_edges {
                    if let Some(allowed) = allowed_edges {
                        if !allowed.contains(&edge_idx) {
                            continue;
                        }
                    }
                    let edge = graph.edge(edge_idx);
                    if (edge.payload.allowed_modes & (allowed_modes | fallback_modes)) == 0 {
                        continue;
                    }

                    let v = if edge.from == u { edge.to } else { edge.from };

                    if let Some((min_lon, min_lat, max_lon, max_lat)) = bounding_box {
                        let p = graph.node(v).payload.point;
                        if p.x() < min_lon || p.x() > max_lon || p.y() < min_lat || p.y() > max_lat
                        {
                            continue;
                        }
                    }

                    let mut cost = get_edge_cost(edge);
                    if (edge.payload.allowed_modes & allowed_modes) == 0 {
                        cost *= 10.0;
                    }
                    let tentative_g = current_g + cost;

                    if tentative_g < *ctx.g_fwd.get(&v).unwrap_or(&f64::INFINITY) {
                        ctx.g_fwd.insert(v, tentative_g);
                        ctx.open_fwd.push(State {
                            cost: tentative_g + h_fwd(v),
                            g: tentative_g,
                            node: v,
                        });

                        if let Some(&g_b) = ctx.g_bwd.get(&v) {
                            let dist = tentative_g + g_b;
                            if dist < mu {
                                mu = dist;
                            }
                        }
                    }
                }
            }
        } else {
            if let Some(State {
                cost: _,
                g,
                node: u,
            }) = ctx.open_bwd.pop()
            {
                let current_g = *ctx.g_bwd.get(&u).unwrap_or(&f64::INFINITY);

                if g > current_g || current_g + h_bwd(u) >= mu {
                    continue;
                }

                for &edge_idx in &graph.node(u).in_edges {
                    if let Some(allowed) = allowed_edges {
                        if !allowed.contains(&edge_idx) {
                            continue;
                        }
                    }
                    let edge = graph.edge(edge_idx);
                    if (edge.payload.allowed_modes & (allowed_modes | fallback_modes)) == 0 {
                        continue;
                    }

                    let v = if edge.from == u { edge.to } else { edge.from };

                    if let Some((min_lon, min_lat, max_lon, max_lat)) = bounding_box {
                        let p = graph.node(v).payload.point;
                        if p.x() < min_lon || p.x() > max_lon || p.y() < min_lat || p.y() > max_lat
                        {
                            continue;
                        }
                    }

                    let mut cost = get_edge_cost(edge);
                    if (edge.payload.allowed_modes & allowed_modes) == 0 {
                        cost *= 10.0;
                    }
                    let tentative_g = current_g + cost;

                    if tentative_g < *ctx.g_bwd.get(&v).unwrap_or(&f64::INFINITY) {
                        ctx.g_bwd.insert(v, tentative_g);
                        ctx.open_bwd.push(State {
                            cost: tentative_g + h_bwd(v),
                            g: tentative_g,
                            node: v,
                        });

                        if let Some(&g_f) = ctx.g_fwd.get(&v) {
                            let dist = g_f + tentative_g;
                            if dist < mu {
                                mu = dist;
                            }
                        }
                    }
                }
            }
        }
    }

    if mu < f64::INFINITY {
        return Some(mu);
    }

    None
}

#[derive(Copy, Clone, PartialEq)]
struct MultiState {
    cost: f64,
    g: f64,
    node: NodeIndex,
    start_node: NodeIndex,
}

impl Eq for MultiState {}

impl Ord for MultiState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for MultiState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn multi_target_dijkstra(
    graph: &Graph<NodePL, EdgePL>,
    starts: &[(NodeIndex, f64)],
    targets: &[NodeIndex],
    allowed_modes: u8,
    fallback_modes: u8,
    allowed_edges: Option<&AHashSet<usize>>,
    preferred_match: Option<&TransitMatch>,
    bounding_box: Option<(f64, f64, f64, f64)>,
    max_cost: f64,
) -> AHashMap<NodeIndex, (f64, NodeIndex)> {
    let mut open = BinaryHeap::new();
    let mut g_score: AHashMap<NodeIndex, (f64, NodeIndex)> = AHashMap::new();

    for &(start_node, init_cost) in starts {
        if init_cost >= max_cost {
            continue;
        }
        // Keep the best initial cost if there are duplicates
        if let Some(&(best_g, _)) = g_score.get(&start_node) {
            if init_cost >= best_g {
                continue;
            }
        }
        g_score.insert(start_node, (init_cost, start_node));
        open.push(MultiState {
            cost: init_cost,
            g: init_cost,
            node: start_node,
            start_node,
        });
    }

    let mut target_set: AHashSet<NodeIndex> = targets.iter().copied().collect();
    let mut found_targets = 0;
    let target_count = target_set.len();

    let get_edge_cost = |edge: &crate::graph::Edge<EdgePL>| -> f64 {
        let mut cost = edge.payload.cost as f64;
        if let Some(pm) = preferred_match {
            for &line_id in &edge.payload.lines {
                let Some(line) = graph.transit_info(line_id) else {
                    continue;
                };
                let mut matches = false;
                if let Some(target) = &pm.short_name {
                    let line_name = &line.short_name;
                    if contains_ignore_case(line_name, target)
                        || target.contains(&line_name.to_lowercase())
                    {
                        matches = true;
                    }
                }
                if !matches {
                    if let Some(target) = &pm.long_name {
                        let line_name = &line.short_name;
                        if contains_ignore_case(line_name, target)
                            || target.contains(&line_name.to_lowercase())
                        {
                            matches = true;
                        }
                    }
                }
                if matches {
                    cost *= 0.2;
                    break;
                }
            }
        }
        cost
    };

    while let Some(MultiState {
        cost: _,
        g,
        node: u,
        start_node,
    }) = open.pop()
    {
        let current_g = g_score.get(&u).map(|&(c, _)| c).unwrap_or(f64::INFINITY);
        if g > current_g || g >= max_cost {
            continue;
        }

        if target_set.remove(&u) {
            found_targets += 1;
            if found_targets == target_count {
                break;
            }
        }

        for &edge_idx in &graph.node(u).out_edges {
            if let Some(allowed) = allowed_edges {
                if !allowed.contains(&edge_idx) {
                    continue;
                }
            }
            let edge = graph.edge(edge_idx);
            if (edge.payload.allowed_modes & (allowed_modes | fallback_modes)) == 0 {
                continue;
            }

            let v = if edge.from == u { edge.to } else { edge.from };

            if let Some((min_lon, min_lat, max_lon, max_lat)) = bounding_box {
                let p = graph.node(v).payload.point;
                if p.x() < min_lon || p.x() > max_lon || p.y() < min_lat || p.y() > max_lat {
                    continue;
                }
            }

            let mut step_cost = get_edge_cost(edge);
            if (edge.payload.allowed_modes & allowed_modes) == 0 {
                step_cost *= 10.0;
            }
            let tentative_g = g + step_cost;

            if tentative_g < max_cost {
                let current_v_g = g_score.get(&v).map(|&(c, _)| c).unwrap_or(f64::INFINITY);
                if tentative_g < current_v_g {
                    g_score.insert(v, (tentative_g, start_node));
                    open.push(MultiState {
                        cost: tentative_g, // No heuristic for multi-target, so f = g
                        g: tentative_g,
                        node: v,
                        start_node,
                    });
                }
            }
        }
    }

    let mut result = AHashMap::new();
    for &t in targets {
        if let Some(&res) = g_score.get(&t) {
            result.insert(t, res);
        }
    }
    result
}
