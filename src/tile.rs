//! Tile-based graph loading for memory-efficient bus/coach shape matching.
//!
//! Instead of loading the entire OSM graph into memory, we partition
//! the world into tiles (~50km cells) and load only what's needed for
//! each stop pair.

use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use geo::{LineString, Point};
use lru::LruCache;
use rstar::RTree;
use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};

use crate::graph::{EdgeIndex, EdgePL, Graph, MODE_BUS, NodeIndex, NodePL};
use crate::osm_load::SpatialNode;

/// Tile size in degrees. 0.5° ≈ 50km at mid-latitudes.
pub const TILE_SIZE: f64 = 0.5;

/// Buffer in degrees to add around tile edges for seamless routing.
/// ~200m at mid-latitudes.
const TILE_BUFFER: f64 = 0.002;

/// Tile coordinate (x = longitude bucket, y = latitude bucket).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TileCoord {
    pub x: i32,
    pub y: i32,
}

impl TileCoord {
    /// Create tile coordinate from a geographic point.
    pub fn from_point(lon: f64, lat: f64) -> Self {
        Self {
            x: (lon / TILE_SIZE).floor() as i32,
            y: (lat / TILE_SIZE).floor() as i32,
        }
    }

    /// Get the bounding box for this tile (with buffer).
    pub fn bbox(&self) -> (f64, f64, f64, f64) {
        let min_lon = (self.x as f64) * TILE_SIZE - TILE_BUFFER;
        let min_lat = (self.y as f64) * TILE_SIZE - TILE_BUFFER;
        let max_lon = ((self.x + 1) as f64) * TILE_SIZE + TILE_BUFFER;
        let max_lat = ((self.y + 1) as f64) * TILE_SIZE + TILE_BUFFER;
        (min_lon, min_lat, max_lon, max_lat)
    }

    /// Get this tile and all 8 neighbors.
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

/// Compute the corridor of tiles between two points.
/// Uses Bresenham-like line algorithm on tile grid.
pub fn compute_corridor_tiles(p1: Point<f64>, p2: Point<f64>) -> Vec<TileCoord> {
    let t1 = TileCoord::from_point(p1.x(), p1.y());
    let t2 = TileCoord::from_point(p2.x(), p2.y());

    if t1 == t2 {
        return vec![t1];
    }

    let mut tiles = Vec::new();
    let mut visited = AHashSet::new();

    // Simple line walking: sample points along the line
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

    // Add neighbors along the corridor for better coverage
    let corridor_with_neighbors: Vec<TileCoord> = tiles
        .iter()
        .flat_map(|t| {
            // Include tile and immediate neighbors perpendicular to path
            vec![
                *t,
                TileCoord { x: t.x - 1, y: t.y },
                TileCoord { x: t.x + 1, y: t.y },
                TileCoord { x: t.x, y: t.y - 1 },
                TileCoord { x: t.x, y: t.y + 1 },
            ]
        })
        .collect();

    // Deduplicate while preserving rough order
    let mut final_tiles = Vec::new();
    let mut seen = AHashSet::new();
    for t in corridor_with_neighbors {
        if seen.insert(t) {
            final_tiles.push(t);
        }
    }

    final_tiles
}

/// Data for a single tile - serializable for disk caching.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct TileData {
    pub graph: Graph<NodePL, EdgePL>,
    pub spatial_nodes: Vec<SpatialNode>, // Stored flat for serialization
    pub osm_node_to_graph_idx: AHashMap<i64, NodeIndex>,
    #[serde(skip)]
    pub spatial_tree: Option<RTree<SpatialNode>>, // Built on load
}

impl TileData {
    pub fn new() -> Self {
        Self {
            graph: Graph::new(),
            spatial_nodes: Vec::new(),
            spatial_tree: None,
            osm_node_to_graph_idx: AHashMap::new(),
        }
    }

    /// Rebuild spatial tree from nodes (call after deserialization)
    pub fn rebuild_spatial_tree(&mut self) {
        self.spatial_tree = Some(RTree::bulk_load(self.spatial_nodes.clone()));
    }

    /// Get spatial tree, building if needed
    pub fn get_spatial_tree(&mut self) -> &RTree<SpatialNode> {
        if self.spatial_tree.is_none() {
            self.rebuild_spatial_tree();
        }
        self.spatial_tree.as_ref().unwrap()
    }
}

/// Merged data from multiple tiles for pathfinding.
pub struct MergedTileData {
    pub graph: Graph<NodePL, EdgePL>,
    pub spatial_tree: RTree<SpatialNode>,
}

use std::sync::Arc;

/// LRU cache for tiles with optional disk persistence.
pub struct TileCache {
    cache: LruCache<TileCoord, Arc<TileData>>,
    osm_path: std::path::PathBuf,
    disk_cache_dir: Option<std::path::PathBuf>,
    use_disk_cache: bool,
    is_split_dir: bool,
}

impl TileCache {
    /// Create a new tile cache with in-memory LRU only.
    pub fn new(osm_path: &Path, capacity: usize) -> Self {
        Self {
            cache: LruCache::new(NonZeroUsize::new(capacity).unwrap()),
            osm_path: osm_path.to_path_buf(),
            disk_cache_dir: None,
            use_disk_cache: false,
            is_split_dir: false,
        }
    }

    /// Create a tile cache with disk persistence in /tmp.
    pub fn new_with_disk_cache(osm_path: &Path, capacity: usize) -> Result<Self> {
        // Create a unique cache directory based on OSM file path hash
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();
        osm_path.hash(&mut hasher);
        let hash = hasher.finish();

        let cache_dir = std::path::PathBuf::from(format!("/tmp/pfaedle-tiles-{:x}", hash));
        std::fs::create_dir_all(&cache_dir)
            .with_context(|| format!("Failed to create tile cache dir: {:?}", cache_dir))?;

        println!("Tile disk cache: {:?}", cache_dir);

        Ok(Self {
            cache: LruCache::new(NonZeroUsize::new(capacity).unwrap()),
            osm_path: osm_path.to_path_buf(),
            disk_cache_dir: Some(cache_dir),
            use_disk_cache: true,
            is_split_dir: false,
        })
    }

    /// Get cache file path for a tile.
    fn tile_cache_path(&self, coord: TileCoord) -> Option<std::path::PathBuf> {
        self.disk_cache_dir
            .as_ref()
            .map(|dir| dir.join(format!("tile_{}_{}.bin", coord.x, coord.y)))
    }

    /// Check if a tile exists on disk.
    fn tile_exists_on_disk(&self, coord: TileCoord) -> bool {
        self.tile_cache_path(coord).map_or(false, |p| p.exists())
    }

    /// Mark a tile as built (create empty marker file).
    fn mark_tile_built(&self, coord: TileCoord) -> Result<()> {
        if let Some(path) = self.tile_cache_path(coord) {
            // Just create a marker file - we rebuild from OSM each time
            // but this helps track progress
            std::fs::write(&path, b"built")?;
        }
        Ok(())
    }

    /// Get number of tiles currently in cache.
    pub fn len(&self) -> usize {
        self.cache.len()
    }

    /// Build or retrieve a tile (with disk caching if enabled).
    pub fn new_with_split_dir(split_dir: &Path, capacity: usize) -> Result<Self> {
        Ok(Self {
            cache: LruCache::new(std::num::NonZeroUsize::new(capacity).unwrap()),
            osm_path: PathBuf::new(), // Not used for building, but maybe need to keep?
            disk_cache_dir: Some(split_dir.to_path_buf()),
            use_disk_cache: true,
            is_split_dir: true,
        })
    }

    pub fn get(&mut self, coord: TileCoord) -> Result<Arc<TileData>> {
        if let Some(tile) = self.cache.get(&coord) {
            return Ok(tile.clone());
        }

        // Load from disk
        let tile = if self.is_split_dir {
            self.load_from_split_dir(coord)?
        } else if self.tile_exists_on_disk(coord) {
            self.load_tile_from_disk(coord)?
        } else {
            let tile = self.build_tile(coord)?;
            if self.use_disk_cache {
                self.save_tile_to_disk(coord, &tile)?;
            }
            tile
        };

        let tile_arc = Arc::new(tile);
        self.cache.put(coord, tile_arc.clone());
        Ok(tile_arc)
    }

    fn load_from_split_dir(&self, coord: TileCoord) -> Result<TileData> {
        // Load nodes and ways from split buckets
        let node_path = self
            .disk_cache_dir
            .as_ref()
            .unwrap()
            .join(format!("tile_{}_{}.nodes.bin", coord.x, coord.y));
        let way_path = self
            .disk_cache_dir
            .as_ref()
            .unwrap()
            .join(format!("tile_{}_{}.ways.bin", coord.x, coord.y));

        // If files don't exist, tile is empty
        if !node_path.exists() && !way_path.exists() {
            return Ok(TileData {
                graph: Graph::new(),
                spatial_nodes: Vec::new(),
                spatial_tree: None,
                osm_node_to_graph_idx: AHashMap::new(),
            });
        }

        let mut graph = Graph::new();
        let mut osm_node_to_graph_idx = AHashMap::new();

        use crate::osm_split::TileItem;
        use osmpbfreader::Tags;

        // 1. Read Nodes
        if node_path.exists() {
            let f = std::fs::File::open(&node_path)?;
            let mut reader = std::io::BufReader::new(f);

            loop {
                let mut len_buf = [0u8; 4];
                match std::io::Read::read_exact(&mut reader, &mut len_buf) {
                    Ok(_) => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
                    Err(e) => return Err(e.into()),
                }
                let len = u32::from_le_bytes(len_buf) as usize;
                let mut buf = vec![0u8; len];
                std::io::Read::read_exact(&mut reader, &mut buf)?;

                let item: TileItem = bincode::deserialize(&buf)?;
                if let TileItem::Node(n) = item {
                    let idx = graph.add_node(NodePL {
                        point: Point::new(n.lon, n.lat),
                    });
                    osm_node_to_graph_idx.insert(n.id, idx);
                }
            }
        }

        // 2. Read Ways and Build Edges
        let mut spatial_nodes = Vec::with_capacity(osm_node_to_graph_idx.len());
        // We'll collect spatial nodes after graph is built, or build incrementally.
        // Actually, spatial nodes in TileData are just all nodes usually, or specific ones?
        // In OsmBuilder, it filters by mode. Here, we can include all nodes or filter.
        // For simplicity, let's include all nodes that end up in the graph.

        if way_path.exists() {
            let f = std::fs::File::open(&way_path)?;
            let mut reader = std::io::BufReader::new(f);

            loop {
                let mut len_buf = [0u8; 4];
                match std::io::Read::read_exact(&mut reader, &mut len_buf) {
                    Ok(_) => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
                    Err(e) => return Err(e.into()),
                }
                let len = u32::from_le_bytes(len_buf) as usize;
                let mut buf = vec![0u8; len];
                std::io::Read::read_exact(&mut reader, &mut buf)?;

                let item: TileItem = bincode::deserialize(&buf)?;
                if let TileItem::Way(w) = item {
                    // Resolve Nodes
                    let mut way_indices = Vec::with_capacity(w.refs.len());
                    for nid in &w.refs {
                        if let Some(&idx) = osm_node_to_graph_idx.get(nid) {
                            way_indices.push(idx);
                        }
                    }

                    if way_indices.len() > 1 {
                        // We need tags to determine EdgePL properties.
                        // We need access to OsmBuilder helpers or replicate logic.
                        // Logic is simple enough to replicate or pull helpers into a utils module.
                        // OsmBuilder implementation is inside `impl OsmBuilder` but some helpers are static.
                        // Let's assume we can access them as `OsmBuilder::parse_oneway` etc if they are public?
                        // They are private in `osm_load.rs`.
                        // I should have made them public.
                        // For now, I'll inline simplified logic or standard OSM logic.

                        let tags: Tags = w
                            .tags
                            .into_iter()
                            .map(|(k, v)| (k.into(), v.into()))
                            .collect();

                        // Parse properties
                        // Level
                        let level = tags
                            .get("level")
                            .map(|s| s.as_str())
                            .and_then(|s| s.parse::<f32>().ok())
                            .unwrap_or(0.0);

                        // Oneway
                        let oneway = if tags
                            .get("oneway")
                            .map(|s| s.as_str())
                            .map_or(false, |s| s == "yes" || s == "true" || s == "1")
                        {
                            1
                        } else if tags
                            .get("oneway")
                            .map(|s| s.as_str())
                            .map_or(false, |s| s == "-1")
                        {
                            2
                        } else {
                            0
                        };

                        // Preferred Direction (simplified)
                        let preferred_direction = 0; // Default

                        // Modes (simplified for bus matching - basically allow BUS for everything infra)
                        let mut modes = MODE_BUS;
                        // If checking specific route types:
                        if tags.contains_key("railway") {
                            modes |= crate::graph::MODE_RAIL;
                        }
                        // etc.

                        // Cost calculation
                        // We need length.

                        for i in 0..way_indices.len() - 1 {
                            let u = way_indices[i];
                            let v = way_indices[i + 1];
                            let p1 = graph.nodes[u].payload.point;
                            let p2 = graph.nodes[v].payload.point;
                            let geom = LineString::new(vec![p1.into(), p2.into()]);

                            let mut edge_pl = EdgePL::new();
                            edge_pl.geometry = geom;
                            edge_pl.level = level as i32;
                            edge_pl.oneway = oneway;
                            edge_pl.preferred_direction = preferred_direction;
                            edge_pl.allowed_modes = modes;
                            edge_pl.osmid = w.id;
                            edge_pl.cost = (edge_pl.length() * 100.0) as u32; // Simple cost = length * 100 (cm approx)

                            graph.add_edge(u, v, edge_pl);
                        }
                    }
                }
            }
        }

        // Post-processing (reverse edges for two-way)
        let existing_edges: Vec<_> = graph
            .edges
            .iter()
            .map(|e| (e.from, e.to, e.payload.clone()))
            .collect();
        for (u, v, pl) in existing_edges {
            if pl.oneway == 0 {
                // Add reverse
                let mut rev_pl = pl.clone();
                rev_pl.geometry = LineString::new(pl.geometry.0.iter().rev().cloned().collect());
                graph.add_edge(v, u, rev_pl);
            }
            // Handle oneway reverse logic if needed (e.g. oneway=-1 was handled above?)
        }

        // Build Spatial Nodes
        for (idx, node) in graph.nodes.iter().enumerate() {
            let p = &node.payload.point;
            spatial_nodes.push(SpatialNode {
                index: idx,
                point: [p.x(), p.y()],
                modes: MODE_BUS, // Assume accessible
            });
        }

        let spatial_tree = if !spatial_nodes.is_empty() {
            Some(RTree::bulk_load(spatial_nodes.clone()))
        } else {
            None
        };

        Ok(TileData {
            graph,
            spatial_nodes,
            spatial_tree,
            osm_node_to_graph_idx,
        })
    }

    /// Build tile data for a specific coordinate from OSM.
    fn build_tile(&self, coord: TileCoord) -> Result<TileData> {
        use crate::osm_load::OsmBuilder;
        use gtfs_structures::RouteType;

        let bbox = coord.bbox();

        // Use only bus mode for tile building
        let mut used_types = AHashSet::new();
        used_types.insert(RouteType::Bus);

        // Load OSM data for this tile's bounding box
        let osm_data = OsmBuilder::read(&self.osm_path, &used_types, Some(bbox), false)
            .with_context(|| format!("Failed to load tile {:?}", coord))?;

        // Extract spatial nodes from the tree
        let spatial_nodes: Vec<SpatialNode> = osm_data
            .spatial_tree
            .map(|tree| tree.iter().cloned().collect())
            .unwrap_or_default();

        let spatial_tree = Some(RTree::bulk_load(spatial_nodes.clone()));

        Ok(TileData {
            graph: osm_data.graph,
            spatial_nodes,
            spatial_tree,
            osm_node_to_graph_idx: AHashMap::new(),
        })
    }

    /// Save tile to disk using bincode.
    fn save_tile_to_disk(&self, coord: TileCoord, tile: &TileData) -> Result<()> {
        if let Some(path) = self.tile_cache_path(coord) {
            let data = bincode::serialize(tile)
                .with_context(|| format!("Failed to serialize tile {:?}", coord))?;
            std::fs::write(&path, data)
                .with_context(|| format!("Failed to write tile {:?}", path))?;
        }
        Ok(())
    }

    /// Load tile from disk using bincode.
    fn load_tile_from_disk(&self, coord: TileCoord) -> Result<TileData> {
        let path = self
            .tile_cache_path(coord)
            .ok_or_else(|| anyhow::anyhow!("No disk cache configured"))?;

        let data =
            std::fs::read(&path).with_context(|| format!("Failed to read tile {:?}", path))?;

        let mut tile: TileData = bincode::deserialize(&data)
            .with_context(|| format!("Failed to deserialize tile {:?}", coord))?;

        // Rebuild spatial tree after deserialization
        tile.rebuild_spatial_tree();

        Ok(tile)
    }

    /// Load and merge tiles needed for a segment between two points.
    pub fn get_for_segment(&mut self, p1: Point<f64>, p2: Point<f64>) -> Result<MergedTileData> {
        let t1 = TileCoord::from_point(p1.x(), p1.y());
        let t2 = TileCoord::from_point(p2.x(), p2.y());

        // Collect tiles needed
        let tiles_needed: Vec<TileCoord> = if t1 == t2 {
            // Same tile: just that tile + neighbors
            t1.with_neighbors().to_vec()
        } else if (t1.x - t2.x).abs() <= 1 && (t1.y - t2.y).abs() <= 1 {
            // Adjacent tiles: both tiles + their neighbors
            let mut all = t1.with_neighbors().to_vec();
            all.extend(t2.with_neighbors());
            all.sort_by_key(|t| (t.x, t.y));
            all.dedup();
            all
        } else {
            // Distant tiles: compute corridor
            compute_corridor_tiles(p1, p2)
        };

        self.merge_tiles(&tiles_needed)
    }

    /// Merge multiple tiles into one unified graph.
    pub fn merge_tiles(&mut self, coords: &[TileCoord]) -> Result<MergedTileData> {
        let mut merged_graph = Graph::new();
        let mut all_spatial_nodes = Vec::new();

        // Track node positions to merge boundary nodes
        let mut pos_to_node: AHashMap<(i64, i64), NodeIndex> = AHashMap::new();

        for &coord in coords {
            let tile = self.get(coord)?;

            // Map old node indices to new merged indices
            let mut old_to_new: AHashMap<NodeIndex, NodeIndex> = AHashMap::new();

            for (old_idx, node) in tile.graph.nodes.iter().enumerate() {
                let p = &node.payload.point;
                // Quantize position to detect same nodes across tiles
                let key = ((p.x() * 1_000_000.0) as i64, (p.y() * 1_000_000.0) as i64);

                let new_idx = if let Some(&existing) = pos_to_node.get(&key) {
                    existing
                } else {
                    let idx = merged_graph.add_node(node.payload.clone());
                    pos_to_node.insert(key, idx);
                    idx
                };

                old_to_new.insert(old_idx, new_idx);
            }

            // Add edges with remapped node indices
            for edge in &tile.graph.edges {
                if let (Some(&from), Some(&to)) =
                    (old_to_new.get(&edge.from), old_to_new.get(&edge.to))
                {
                    merged_graph.add_edge(from, to, edge.payload.clone());
                }
            }
        }

        // Build spatial tree for merged graph
        for (idx, node) in merged_graph.nodes.iter().enumerate() {
            let p = &node.payload.point;
            all_spatial_nodes.push(SpatialNode {
                index: idx,
                point: [p.x(), p.y()],
                modes: MODE_BUS,
            });
        }

        Ok(MergedTileData {
            graph: merged_graph,
            spatial_tree: RTree::bulk_load(all_spatial_nodes),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tile_coord_from_point() {
        let t = TileCoord::from_point(-122.4, 37.8);
        assert_eq!(t.x, -245);
        assert_eq!(t.y, 75);
    }

    #[test]
    fn test_tile_coord_same_tile() {
        let t1 = TileCoord::from_point(-122.4, 37.8);
        let t2 = TileCoord::from_point(-122.35, 37.85);
        assert_eq!(t1, t2);
    }

    #[test]
    fn test_corridor_tiles_same() {
        let p1 = Point::new(-122.4, 37.8);
        let p2 = Point::new(-122.35, 37.85);
        let corridor = compute_corridor_tiles(p1, p2);
        assert_eq!(corridor.len(), 5); // Self + 4 neighbors
    }

    #[test]
    fn test_corridor_tiles_distant() {
        let p1 = Point::new(-122.0, 37.0);
        let p2 = Point::new(-120.0, 39.0); // ~200km apart
        let corridor = compute_corridor_tiles(p1, p2);
        assert!(corridor.len() > 10);
    }
}
