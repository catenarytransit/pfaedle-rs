//! Disk-backed bus graph loading with a single active merged graph.
//!
//! Upstream pfaedle builds one mode-specific graph and an index that references
//! that graph. The old Rust tiled path retained source tile graphs, per-tile
//! R-trees, and up to four fully cloned merged graphs at once. This loader keeps
//! the tiled on-disk preprocessing but materializes only the current merged raw
//! graph, collapses/finalizes it with the upstream algorithm, and then lets the
//! matcher build one edge R-tree over that graph.

use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use geo::{LineString, Point};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::graph::{EdgePL, Graph, MODE_BUS, NodeIndex, NodePL, TransitInfoInterner};
use crate::osm_load::LightOsmData;
use crate::upstream_graph::{
    NODE_FLAG_RESTRICTION, apply_one_way_penalty, bus_speed_mps, coach_speed_mps, collapse_edges,
    write_components, write_other_direction_edges,
};

/// Tile size in degrees. Kept identical to the existing streaming splitter.
pub const TILE_SIZE: f64 = 0.2;

/// Upstream has a common `[bus, coach]` OSM filter and a `[coach]` section
/// that overrides routing levels/speeds. Keep those graph profiles separate
/// while reusing the same split files.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum RoadProfile {
    Bus,
    Coach,
}

impl RoadProfile {
    fn level(self, way: &crate::osm_split::WayData) -> i32 {
        match self {
            RoadProfile::Bus => way.bus_level as i32,
            RoadProfile::Coach => way.coach_level as i32,
        }
    }

    fn speed_mps(self, level: i32) -> f64 {
        match self {
            RoadProfile::Bus => bus_speed_mps(level),
            RoadProfile::Coach => coach_speed_mps(level),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TileCoord {
    pub x: i32,
    pub y: i32,
}

impl TileCoord {
    pub fn from_point(lon: f64, lat: f64) -> Self {
        Self {
            x: (lon / TILE_SIZE).floor() as i32,
            y: (lat / TILE_SIZE).floor() as i32,
        }
    }

    pub fn with_neighbors(&self) -> [TileCoord; 9] {
        [
            *self,
            TileCoord {
                x: self.x - 1,
                y: self.y - 1,
            },
            TileCoord {
                x: self.x,
                y: self.y - 1,
            },
            TileCoord {
                x: self.x + 1,
                y: self.y - 1,
            },
            TileCoord {
                x: self.x - 1,
                y: self.y,
            },
            TileCoord {
                x: self.x + 1,
                y: self.y,
            },
            TileCoord {
                x: self.x - 1,
                y: self.y + 1,
            },
            TileCoord {
                x: self.x,
                y: self.y + 1,
            },
            TileCoord {
                x: self.x + 1,
                y: self.y + 1,
            },
        ]
    }
}

fn cross_neighbors(t: TileCoord) -> [TileCoord; 5] {
    [
        t,
        TileCoord { x: t.x - 1, y: t.y },
        TileCoord { x: t.x + 1, y: t.y },
        TileCoord { x: t.x, y: t.y - 1 },
        TileCoord { x: t.x, y: t.y + 1 },
    ]
}

pub fn compute_corridor_tiles(p1: Point<f64>, p2: Point<f64>) -> Vec<TileCoord> {
    let t1 = TileCoord::from_point(p1.x(), p1.y());
    let t2 = TileCoord::from_point(p2.x(), p2.y());

    if t1 == t2 {
        return cross_neighbors(t1).to_vec();
    }

    let mut tiles = Vec::new();
    let mut visited = AHashSet::new();
    let steps = ((t2.x - t1.x).abs().max((t2.y - t1.y).abs()) + 1) * 2;
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let lon = p1.x() + t * (p2.x() - p1.x());
        let lat = p1.y() + t * (p2.y() - p1.y());
        let tile = TileCoord::from_point(lon, lat);
        if visited.insert(tile) {
            tiles.push(tile);
        }
    }

    let mut result = Vec::new();
    let mut seen = AHashSet::new();
    for tile in tiles.iter().flat_map(|tile| cross_neighbors(*tile)) {
        if seen.insert(tile) {
            result.push(tile);
        }
    }
    result
}

pub fn compute_route_tiles(stop_coords: &[Point<f64>]) -> Vec<TileCoord> {
    if stop_coords.len() < 2 {
        return stop_coords
            .first()
            .map(|point| {
                TileCoord::from_point(point.x(), point.y())
                    .with_neighbors()
                    .to_vec()
            })
            .unwrap_or_default();
    }

    let mut all_tiles = AHashSet::new();
    for window in stop_coords.windows(2) {
        let p1 = window[0];
        let p2 = window[1];
        let t1 = TileCoord::from_point(p1.x(), p1.y());
        let t2 = TileCoord::from_point(p2.x(), p2.y());

        if t1 == t2 {
            all_tiles.extend(t1.with_neighbors());
        } else if (t1.x - t2.x).abs() <= 1 && (t1.y - t2.y).abs() <= 1 {
            all_tiles.extend(t1.with_neighbors());
            all_tiles.extend(t2.with_neighbors());
        } else {
            all_tiles.extend(compute_corridor_tiles(p1, p2));
        }
    }

    let mut result: Vec<_> = all_tiles.into_iter().collect();
    result.sort_by_key(|tile| (tile.x, tile.y));
    result
}

pub struct TileCache {
    split_dir: PathBuf,
    light_osm: Arc<LightOsmData>,
}

impl TileCache {
    pub fn new_with_split_dir(split_dir: &Path, light_osm: Arc<LightOsmData>) -> Result<Self> {
        Ok(Self {
            split_dir: split_dir.to_path_buf(),
            light_osm,
        })
    }

    fn tile_path(&self, coord: TileCoord) -> PathBuf {
        self.split_dir
            .join(format!("tile_{}_{}.bin", coord.x, coord.y))
    }

    fn read_items<F>(&self, coord: TileCoord, mut visitor: F) -> Result<()>
    where
        F: FnMut(crate::osm_split::TileItem) -> Result<()>,
    {
        let path = self.tile_path(coord);
        if !path.exists() {
            return Ok(());
        }

        let file = std::fs::File::open(&path)
            .with_context(|| format!("Failed to open split tile {:?}", path))?;
        let mut reader = std::io::BufReader::new(file);
        let mut buf = Vec::with_capacity(1024);

        loop {
            let mut len_buf = [0u8; 4];
            match std::io::Read::read_exact(&mut reader, &mut len_buf) {
                Ok(()) => {}
                Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => break,
                Err(error) => return Err(error.into()),
            }
            let len = u32::from_le_bytes(len_buf) as usize;
            buf.resize(len, 0);
            std::io::Read::read_exact(&mut reader, &mut buf)?;
            let item = bincode::deserialize(&buf)?;
            visitor(item)?;
        }
        Ok(())
    }

    fn attach_bus_transit_lines(&self, graph: &mut Graph<NodePL, EdgePL>) {
        let mut interner = TransitInfoInterner::new();
        let mut way_lines: AHashMap<i64, Vec<u32>> = AHashMap::new();

        for edge in &mut graph.edges {
            if edge.payload.osmid == 0 {
                continue;
            }

            let line_ids = way_lines.entry(edge.payload.osmid).or_insert_with(|| {
                let mut ids = Vec::new();
                if let Some(rel_indices) = self.light_osm.way_to_relations.get(&edge.payload.osmid)
                {
                    for &relation_idx in rel_indices {
                        let Some(relation) = self.light_osm.relations.get(relation_idx) else {
                            continue;
                        };
                        let short_name = relation
                            .ref_tag
                            .as_ref()
                            .or(relation.name.as_ref())
                            .cloned()
                            .unwrap_or_default();
                        if short_name.is_empty()
                            && relation.from_str.is_empty()
                            && relation.to_str.is_empty()
                        {
                            continue;
                        }
                        let id = interner.intern(crate::graph::TransitInfo {
                            short_name,
                            from_str: relation.from_str.clone(),
                            to_str: relation.to_str.clone(),
                        });
                        ids.push(id);
                    }
                }
                ids.sort_unstable();
                ids.dedup();
                ids
            });
            edge.payload.lines = line_ids.clone();
        }

        graph.transit_lines = interner.into_infos();
    }

    /// Materialize one raw graph for the current bus route group.
    ///
    /// Each tile file is read twice: nodes first, then ways. This deliberately
    /// trades local /tmp I/O for lower RSS, following upstream pfaedle's own
    /// multi-pass OSM reader. No source tile graph or per-tile R-tree is retained.
    pub fn merge_tiles(
        &self,
        coords: &[TileCoord],
        profile: RoadProfile,
    ) -> Result<Graph<NodePL, EdgePL>> {
        let mut graph = Graph::new();
        let mut node_flags: Vec<u8> = Vec::new();
        let mut osm_node_to_graph: AHashMap<i64, NodeIndex> = AHashMap::new();

        // Pass A: collect/deduplicate nodes directly into the one merged graph.
        for &coord in coords {
            self.read_items(coord, |item| {
                if let crate::osm_split::TileItem::Node(node) = item {
                    if let Some(&idx) = osm_node_to_graph.get(&node.id) {
                        node_flags[idx] |= node.flags;
                    } else {
                        let idx = graph.add_node(NodePL {
                            comp_id: 0,
                            point: Point::new(node.lon, node.lat),
                        });
                        osm_node_to_graph.insert(node.id, idx);
                        node_flags.push(node.flags);
                    }
                }
                Ok(())
            })?;
        }

        // Pass B: add raw forward edges. Split files overlap by design, so use
        // the graph's directed endpoint pair as the same no-multigraph invariant
        // enforced by upstream util::graph::DirGraph.
        for &coord in coords {
            self.read_items(coord, |item| {
                let crate::osm_split::TileItem::Way(way) = item else {
                    return Ok(());
                };

                for refs in way.refs.windows(2) {
                    let (Some(&from), Some(&to)) = (
                        osm_node_to_graph.get(&refs[0]),
                        osm_node_to_graph.get(&refs[1]),
                    ) else {
                        continue;
                    };
                    if from == to
                        || graph.nodes[from]
                            .out_edges
                            .iter()
                            .any(|&edge_idx| graph.edges[edge_idx].to == to)
                    {
                        continue;
                    }

                    let mut payload = EdgePL::new();
                    payload.geometry = LineString::new(vec![
                        graph.nodes[from].payload.point.into(),
                        graph.nodes[to].payload.point.into(),
                    ]);
                    payload.level = profile.level(&way);
                    payload.oneway = way.oneway;
                    payload.allowed_modes = MODE_BUS;
                    payload.osmid = way.id;
                    payload.restriction = way.restriction
                        && ((node_flags[from] & NODE_FLAG_RESTRICTION) != 0
                            || (node_flags[to] & NODE_FLAG_RESTRICTION) != 0);
                    let speed = profile.speed_mps(payload.level).max(0.1);
                    payload.cost = ((payload.length() / speed) * 10.0)
                        .min(u32::MAX as f64)
                        .ceil() as u32;
                    graph.add_edge(from, to, payload);
                }
                Ok(())
            })?;
        }

        // Upstream attaches canonical TransitEdgeLine pointers before
        // collapseEdges(); do the same with compact IDs before collapsing.
        self.attach_bus_transit_lines(&mut graph);

        collapse_edges(&mut graph, &mut node_flags);

        // Upstream writeGeoms() computes cost after collapse. Recompute from the
        // final polyline and the [bus, coach] level speed so rounding and merged
        // geometry exactly follow that ordering.
        for edge in &mut graph.edges {
            let speed = profile.speed_mps(edge.payload.level).max(0.1);
            edge.payload.cost = ((edge.payload.length() / speed) * 10.0)
                .min(u32::MAX as f64)
                .ceil() as u32;
        }

        write_components(&mut graph);
        write_other_direction_edges(&mut graph);
        // Both upstream rail and bus configurations use factor 5.
        apply_one_way_penalty(&mut graph, 5, 300);

        Ok(graph)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_tiles_are_stable_and_deduplicated() {
        let stops = [Point::new(-122.4, 37.8), Point::new(-122.35, 37.85)];
        let tiles = compute_route_tiles(&stops);
        let unique: AHashSet<_> = tiles.iter().copied().collect();
        assert_eq!(tiles.len(), unique.len());
        assert!(!tiles.is_empty());
    }
}
