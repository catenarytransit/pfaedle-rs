//! Graph finalization matching upstream pfaedle's `OsmBuilder` pipeline.
//!
//! Upstream builds a forward graph, protects stations/turning nodes and
//! restriction boundaries, collapses equivalent degree-2 edges, computes
//! components, and only then creates opposite-direction edges. Keeping that
//! ordering is important both for correctness and memory use.

use geo::LineString;
use osmpbfreader::Tags;

use crate::graph::{EdgeIndex, EdgePL, Graph, NodeIndex, NodePL};

pub const NODE_FLAG_STATION: u8 = 1;
pub const NODE_FLAG_TURN_CYCLE: u8 = 2;
pub const NODE_FLAG_RESTRICTION: u8 = 4;

struct WorkEdge {
    from: NodeIndex,
    to: NodeIndex,
    payload: EdgePL,
    active: bool,
}

fn other(edge: &WorkEdge, node: NodeIndex) -> Option<NodeIndex> {
    if edge.from == node {
        Some(edge.to)
    } else if edge.to == node {
        Some(edge.from)
    } else {
        None
    }
}

/// Rust currently permits several transit modes in one graph, while upstream
/// pfaedle builds a graph per MOT configuration. `allowed_modes` therefore has
/// to match as an additional guard: without it a degree-2 node could merge a
/// rail edge into a tram/bus edge, something upstream can never do.
fn edges_similar(a: &WorkEdge, b: &WorkEdge) -> bool {
    if (a.payload.oneway != 0) != (b.payload.oneway != 0) {
        return false;
    }
    if a.payload.level != b.payload.level {
        return false;
    }
    if a.payload.lines != b.payload.lines {
        return false;
    }
    if a.payload.restriction || b.payload.restriction {
        return false;
    }
    if a.payload.allowed_modes != b.payload.allowed_modes {
        return false;
    }

    // Exact upstream guard for two one-way edges: they must meet head-to-tail.
    if a.payload.oneway != 0 && b.payload.oneway != 0 && a.from != b.to && a.to != b.from {
        return false;
    }

    true
}

fn oriented_geometry(edge: &WorkEdge, from: NodeIndex, to: NodeIndex) -> Vec<geo::Coord<f64>> {
    let mut coords = edge.payload.geometry.0.clone();
    if coords.len() < 2 {
        return coords;
    }

    if edge.from == from && edge.to == to {
        coords
    } else if edge.from == to && edge.to == from {
        coords.reverse();
        coords
    } else {
        Vec::new()
    }
}

fn merge_payload(
    first: &WorkEdge,
    second: &WorkEdge,
    via: NodeIndex,
    new_from: NodeIndex,
    new_to: NodeIndex,
) -> EdgePL {
    let mut first_geom = oriented_geometry(first, new_from, via);
    let mut second_geom = oriented_geometry(second, via, new_to);

    if first_geom.is_empty() || second_geom.is_empty() {
        return first.payload.clone();
    }
    if first_geom.last() == second_geom.first() {
        second_geom.remove(0);
    }
    first_geom.extend(second_geom);

    let mut payload = first.payload.clone();
    payload.geometry = LineString::new(first_geom);
    // C++ recomputes cost from the collapsed geometry and routing level. Rust
    // may also have per-segment maxspeed-derived costs, so summing preserves the
    // exact pre-collapse traversal cost while being equivalent for level-based
    // upstream costs.
    payload.cost = first.payload.cost.saturating_add(second.payload.cost);
    // A collapsed C++ edge has no single OSM way identity. Keep the ID only
    // when the two source edges actually came from the same way.
    if first.payload.osmid != second.payload.osmid {
        payload.osmid = 0;
    }
    payload.is_reverse = false;
    payload
}

fn active_incident_edges(
    incident: &[Vec<EdgeIndex>],
    edges: &[WorkEdge],
    node: NodeIndex,
) -> Vec<EdgeIndex> {
    incident[node]
        .iter()
        .copied()
        .filter(|&edge_idx| edges.get(edge_idx).is_some_and(|edge| edge.active))
        .collect()
}

fn has_edge_between(
    incident: &[Vec<EdgeIndex>],
    edges: &[WorkEdge],
    a: NodeIndex,
    b: NodeIndex,
    except_a: EdgeIndex,
    except_b: EdgeIndex,
) -> bool {
    incident[a].iter().copied().any(|edge_idx| {
        if edge_idx == except_a || edge_idx == except_b {
            return false;
        }
        let Some(edge) = edges.get(edge_idx) else {
            return false;
        };
        edge.active && ((edge.from == a && edge.to == b) || (edge.from == b && edge.to == a))
    })
}

/// Collapse the same graph vertices as C++ `OsmBuilder::collapseEdges`.
///
/// The compact Vec-backed Rust graph needs a temporary node remap while it is
/// rebuilt; that remap is released before this function returns.
pub fn collapse_edges(graph: &mut Graph<NodePL, EdgePL>, node_flags: &mut Vec<u8>) {
    let original_node_count = graph.nodes.len();

    let mut edges: Vec<WorkEdge> = graph
        .edges
        .drain(..)
        .map(|edge| WorkEdge {
            from: edge.from,
            to: edge.to,
            payload: edge.payload,
            active: true,
        })
        .collect();
    let mut incident = vec![Vec::<EdgeIndex>::new(); original_node_count];
    for (edge_idx, edge) in edges.iter().enumerate() {
        incident[edge.from].push(edge_idx);
        if edge.to != edge.from {
            incident[edge.to].push(edge_idx);
        }
    }

    // Upstream uses a single `for (auto n : g->getNds())` pass. Do not turn
    // this into a fixed-point simplifier: revisiting an earlier neighbor after a
    // later collapse can remove a vertex that C++ pfaedle would retain.
    for node in 0..original_node_count {
        if node_flags.get(node).copied().unwrap_or(0) != 0 {
            continue;
        }

        let incident_edges = active_incident_edges(&incident, &edges, node);
        if incident_edges.len() != 2 {
            continue;
        }
        let ea_idx = incident_edges[0];
        let eb_idx = incident_edges[1];
        if ea_idx == eb_idx {
            continue;
        }

        let Some(a_other) = other(&edges[ea_idx], node) else {
            continue;
        };
        let Some(b_other) = other(&edges[eb_idx], node) else {
            continue;
        };
        if a_other == b_other {
            continue;
        }

        // Upstream's graph is not a multigraph. If either direction already
        // exists between the two neighbors, this vertex must remain.
        if has_edge_between(&incident, &edges, a_other, b_other, ea_idx, eb_idx) {
            continue;
        }

        if !edges_similar(&edges[ea_idx], &edges[eb_idx]) {
            continue;
        }

        // Mirror the orientation choice in C++ collapseEdges(). For one-way
        // chains this guarantees that the new graph edge follows the permitted
        // direction. For bidirectional chains either orientation is valid.
        let (first_idx, second_idx) =
            if edges[ea_idx].payload.oneway != 0 && a_other != edges[ea_idx].from {
                (eb_idx, ea_idx)
            } else {
                (ea_idx, eb_idx)
            };

        let Some(new_from) = other(&edges[first_idx], node) else {
            continue;
        };
        let Some(new_to) = other(&edges[second_idx], node) else {
            continue;
        };

        let payload = merge_payload(
            &edges[first_idx],
            &edges[second_idx],
            node,
            new_from,
            new_to,
        );

        edges[ea_idx].active = false;
        edges[eb_idx].active = false;

        let new_idx = edges.len();
        edges.push(WorkEdge {
            from: new_from,
            to: new_to,
            payload,
            active: true,
        });
        incident[new_from].push(new_idx);
        if new_to != new_from {
            incident[new_to].push(new_idx);
        }
    }

    let mut active_degree = vec![0usize; original_node_count];
    for edge in &edges {
        if edge.active {
            active_degree[edge.from] += 1;
            if edge.to != edge.from {
                active_degree[edge.to] += 1;
            }
        }
    }

    let old_graph = std::mem::replace(graph, Graph::new());
    let old_nodes = old_graph.nodes;
    let transit_lines = old_graph.transit_lines;

    let mut rebuilt = Graph::new();
    rebuilt.transit_lines = transit_lines;
    // Temporary old->new node map needed only while compacting the Vec-backed
    // Rust graph. It is dropped immediately afterwards; unlike the earlier
    // implementation no old-edge mapping survives graph construction.
    let mut node_map = vec![usize::MAX; original_node_count];
    let mut rebuilt_flags = Vec::new();

    for (old_idx, node) in old_nodes.into_iter().enumerate() {
        let station = node_flags.get(old_idx).copied().unwrap_or(0) & NODE_FLAG_STATION != 0;
        if active_degree[old_idx] == 0 && !station {
            continue;
        }
        let new_idx = rebuilt.add_node(node.payload);
        node_map[old_idx] = new_idx;
        rebuilt_flags.push(node_flags.get(old_idx).copied().unwrap_or(0));
    }

    for edge in edges {
        if !edge.active {
            continue;
        }
        let from = node_map[edge.from];
        let to = node_map[edge.to];
        if from == usize::MAX || to == usize::MAX {
            continue;
        }
        rebuilt.add_edge(from, to, edge.payload);
    }

    *graph = rebuilt;
    *node_flags = rebuilt_flags;
}

/// Add opposite-direction edges exactly after collapse, matching upstream
/// `writeODirEdgs`.
pub fn write_other_direction_edges(graph: &mut Graph<NodePL, EdgePL>) {
    let forward_len = graph.edges.len();

    for edge_idx in 0..forward_len {
        let from = graph.edges[edge_idx].from;
        let to = graph.edges[edge_idx].to;

        if graph.nodes[to]
            .out_edges
            .iter()
            .any(|&candidate_idx| graph.edges[candidate_idx].to == from)
        {
            continue;
        }
        let payload = graph.edges[edge_idx].payload.rev_copy();
        graph.add_edge(to, from, payload);
    }
}

/// Upstream computes components before adding the opposite-direction copies.
pub fn write_components(graph: &mut Graph<NodePL, EdgePL>) {
    let mut comp_id = 0usize;
    let mut visited = vec![false; graph.nodes.len()];
    let mut queue = std::collections::VecDeque::new();

    for start in 0..graph.nodes.len() {
        if visited[start] {
            continue;
        }
        comp_id += 1;
        visited[start] = true;
        queue.push_back(start);

        while let Some(node_idx) = queue.pop_front() {
            graph.nodes[node_idx].payload.comp_id = comp_id;
            for &edge_idx in &graph.nodes[node_idx].out_edges {
                let next = graph.edges[edge_idx].to;
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
            for &edge_idx in &graph.nodes[node_idx].in_edges {
                let next = graph.edges[edge_idx].from;
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }
}

/// Apply upstream `writeOneWayPens()` after reverse edges are present.
/// Costs are stored in tenths of seconds, while the config entry penalty is
/// specified in seconds.
pub fn apply_one_way_penalty(
    graph: &mut Graph<NodePL, EdgePL>,
    factor: u32,
    entry_cost_seconds: u32,
) {
    let entry_cost = entry_cost_seconds.saturating_mul(10);
    for edge in &mut graph.edges {
        if edge.payload.oneway == 2 {
            edge.payload.cost = edge
                .payload
                .cost
                .saturating_mul(factor)
                .saturating_add(entry_cost);
        }
    }
}

pub fn node_flags(tags: &Tags) -> u8 {
    // Exact `[bus, coach]` station + turning-cycle filters used by the disk
    // splitter. Other MOT profiles derive their flags in OsmBuilder.
    let mut flags = 0u8;

    let station = tags.get("public_transport").map(|s| s.as_str()) == Some("stop_position")
        || tags.contains_key("bus_stop")
        || tags.contains_key("stop")
        || tags.get("highway").map(|s| s.as_str()) == Some("bus_stop")
        || tags.get("amenity").map(|s| s.as_str()) == Some("bus_station");
    if station {
        flags |= NODE_FLAG_STATION;
    }

    let turn_cycle = matches!(
        tags.get("highway").map(|s| s.as_str()),
        Some("turning_circle" | "turning_loop" | "mini_roundabout")
    ) || tags.get("junction").map(|s| s.as_str()) == Some("roundabout");
    if turn_cycle {
        flags |= NODE_FLAG_TURN_CYCLE;
    }

    flags
}

fn tag_is(tags: &Tags, key: &str, values: &[&str]) -> bool {
    tags.get(key)
        .is_some_and(|value| values.iter().any(|candidate| value.as_str() == *candidate))
}

/// Equivalent to the [bus, coach] level filters in upstream `pfaedle.cfg`.
/// `OsmFilter::level()` checks levels in ascending order and returns the first
/// matching one, which is reproduced here.
pub fn bus_level(tags: &Tags) -> u8 {
    let highway = tags.get("highway").map(|s| s.as_str());

    if matches!(
        highway,
        Some("trunk" | "trunk_link" | "primary" | "primary_link")
    ) {
        return 1;
    }
    if matches!(highway, Some("secondary" | "secondary_link"))
        || tag_is(tags, "bus", &["yes", "designated"])
        || tag_is(tags, "minibus", &["yes", "designated"])
        || tag_is(tags, "psv", &["yes", "designated"])
        || tag_is(tags, "access", &["psv", "bus"])
        || tag_is(tags, "trolley_wire", &["yes"])
        || tag_is(tags, "trolleywire", &["yes"])
        || tag_is(tags, "trolleybus", &["yes"])
        || tag_is(tags, "trolley_bus", &["yes"])
    {
        return 2;
    }
    if matches!(highway, Some("tertiary" | "tertiary_link")) {
        return 3;
    }
    if matches!(highway, Some("unclassified" | "residential" | "road")) {
        return 4;
    }
    if matches!(highway, Some("living_street" | "pedestrian" | "service"))
        || tag_is(tags, "psv", &["no"])
    {
        return 5;
    }
    if tag_is(tags, "bus", &["no"])
        || tag_is(tags, "service", &["siding", "parking_aisle"])
        || tag_is(tags, "access", &["permissive", "private", "no"])
        || matches!(highway, Some("footway" | "track"))
    {
        return 6;
    }

    0
}

/// Equivalent to upstream's bus one-way + undirected filters. The explicit
/// undirected rules take precedence, just as `OsmFilter::oneway()` does.
pub fn bus_oneway(tags: &Tags) -> u8 {
    let explicitly_two_way = tag_is(
        tags,
        "oneway",
        &["false", "0", "alternating", "reversible", "no"],
    ) || tag_is(tags, "oneway:bus", &["no", "0", "false"])
        || tag_is(tags, "oneway:psv", &["no", "0", "false"])
        || tag_is(tags, "busway", &["opposite_lane", "opposite"])
        || tag_is(tags, "busway:left", &["opposite_lane"])
        || tag_is(tags, "busway:right", &["opposite_lane"])
        || tag_is(tags, "psv", &["opposite_lane", "opposite"])
        || tag_is(tags, "lanes:psv:backward", &["1", "2"])
        || tag_is(tags, "lanes:bus:backward", &["1", "2"])
        || tag_is(tags, "bus:lanes:backward", &["yes", "designated", "1"]);
    if explicitly_two_way {
        return 0;
    }

    if tag_is(tags, "oneway", &["-1"]) {
        return 2;
    }

    if tags.get("junction").map(|s| s.as_str()) == Some("roundabout")
        || tags.get("highway").map(|s| s.as_str()) == Some("motorway")
        || tag_is(tags, "oneway", &["yes", "1", "true"])
        || tag_is(tags, "oneway:bus", &["yes", "1", "true"])
        || tag_is(tags, "oneway:psv", &["yes", "1", "true"])
    {
        return 1;
    }

    0
}

pub fn bus_speed_mps(level: i32) -> f64 {
    // Upstream pfaedle.cfg [bus, coach] defaults, km/h.
    const SPEED_KMH: [f64; 8] = [85.0, 70.0, 55.0, 40.0, 30.0, 20.0, 10.0, 5.0];
    SPEED_KMH[level.clamp(0, 7) as usize] / 3.6
}

/// Upstream `[coach]` overrides the common bus/coach routing levels.
/// `OsmFilter::level()` returns the first matching level.
pub fn coach_level(tags: &Tags) -> u8 {
    let highway = tags.get("highway").map(|s| s.as_str());

    if matches!(highway, Some("motorway" | "motorway_link")) {
        return 0;
    }
    if matches!(highway, Some("trunk" | "trunk_link")) {
        return 1;
    }
    if matches!(highway, Some("primary" | "primary_link")) {
        return 2;
    }
    if matches!(highway, Some("secondary" | "secondary_link")) {
        return 3;
    }
    if matches!(highway, Some("tertiary" | "tertiary_link")) {
        return 4;
    }
    if matches!(
        highway,
        Some("unclassified" | "residential" | "road" | "service")
    ) {
        return 5;
    }
    if matches!(highway, Some("living_street" | "pedestrian")) || tag_is(tags, "psv", &["no"]) {
        return 6;
    }
    if tag_is(tags, "bus", &["no"])
        || tag_is(tags, "service", &["siding", "parking_aisle"])
        || tag_is(tags, "access", &["permissive", "private", "no"])
        || highway == Some("footway")
    {
        return 7;
    }

    0
}

pub fn coach_speed_mps(level: i32) -> f64 {
    // Upstream pfaedle.cfg [coach] defaults, km/h.
    const SPEED_KMH: [f64; 8] = [120.0, 90.0, 65.0, 50.0, 30.0, 20.0, 10.0, 5.0];
    SPEED_KMH[level.clamp(0, 7) as usize] / 3.6
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{MODE_BUS, TransitInfo};
    use geo::Point;

    fn node(graph: &mut Graph<NodePL, EdgePL>, x: f64) -> NodeIndex {
        graph.add_node(NodePL {
            comp_id: 0,
            point: Point::new(x, 0.0),
        })
    }

    fn edge(graph: &mut Graph<NodePL, EdgePL>, a: NodeIndex, b: NodeIndex) {
        let mut payload = EdgePL::new();
        payload.allowed_modes = MODE_BUS;
        payload.geometry = LineString::new(vec![
            graph.nodes[a].payload.point.into(),
            graph.nodes[b].payload.point.into(),
        ]);
        payload.cost = 10;
        graph.add_edge(a, b, payload);
    }

    #[test]
    fn collapses_equivalent_degree_two_chain_before_reverse_edges() {
        let mut graph = Graph::new();
        let a = node(&mut graph, 0.0);
        let b = node(&mut graph, 1.0);
        let c = node(&mut graph, 2.0);
        edge(&mut graph, a, b);
        edge(&mut graph, b, c);
        let mut flags = vec![0; 3];

        collapse_edges(&mut graph, &mut flags);
        write_components(&mut graph);
        write_other_direction_edges(&mut graph);
        apply_one_way_penalty(&mut graph, 5, 300);

        assert_eq!(graph.nodes.len(), 2);
        assert_eq!(graph.edges.len(), 2);
        assert_eq!(graph.edges[0].payload.geometry.0.len(), 3);
        assert_eq!(graph.edges[0].payload.cost, 20);
        assert!(graph.edges[1].payload.is_reverse);
    }

    #[test]
    fn station_and_line_boundaries_are_not_collapsed() {
        let mut graph = Graph::new();
        graph.transit_lines = vec![
            TransitInfo {
                short_name: "A".into(),
                from_str: String::new(),
                to_str: String::new(),
            },
            TransitInfo {
                short_name: "B".into(),
                from_str: String::new(),
                to_str: String::new(),
            },
        ];
        let a = node(&mut graph, 0.0);
        let b = node(&mut graph, 1.0);
        let c = node(&mut graph, 2.0);
        edge(&mut graph, a, b);
        edge(&mut graph, b, c);
        graph.edges[0].payload.lines = vec![0];
        graph.edges[1].payload.lines = vec![1];
        let mut flags = vec![0; 3];

        collapse_edges(&mut graph, &mut flags);
        assert_eq!(graph.nodes.len(), 3);
        assert_eq!(graph.edges.len(), 2);

        let mut station_graph = Graph::new();
        let a = node(&mut station_graph, 0.0);
        let b = node(&mut station_graph, 1.0);
        let c = node(&mut station_graph, 2.0);
        edge(&mut station_graph, a, b);
        edge(&mut station_graph, b, c);
        let mut station_flags = vec![0, NODE_FLAG_STATION, 0];
        collapse_edges(&mut station_graph, &mut station_flags);
        assert_eq!(station_graph.nodes.len(), 3);
    }
}
