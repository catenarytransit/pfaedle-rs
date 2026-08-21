use ahash::{AHashMap, AHashSet};
use anyhow::Result;
use osmpbfreader::{OsmObj, OsmPbfReader};
use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};

use crate::osm_load::{IdentifiedResources, OsmBuilder};
use crate::tile_loader::TileCoord;
use crate::upstream_graph::{
    NODE_FLAG_RESTRICTION, bus_level, bus_oneway, coach_level, node_flags,
};

/// Maximum bytes to accumulate per tile before flushing (8MB).
const MAX_TILE_BUFFER_SIZE: usize = 8 * 1024 * 1024;

/// Splitter to distribute nodes and ways into tiles.
pub struct OsmSplitter {
    osm_path: PathBuf,
    out_dir: PathBuf,
}

impl OsmSplitter {
    pub fn new(osm_path: &Path, out_dir: &Path) -> Result<Self> {
        if !out_dir.exists() {
            std::fs::create_dir_all(out_dir)?;
        }
        Ok(Self {
            osm_path: osm_path.to_path_buf(),
            out_dir: out_dir.to_path_buf(),
        })
    }

    /// Split the OSM file into disk buckets used by the bus graph loader.
    ///
    /// Node flags and the way routing properties are derived from the same
    /// station/turn-cycle/level/one-way rules used by upstream pfaedle before
    /// `collapseEdges()`. Raw OSM tag maps are still not persisted.
    pub fn split_pbf(&self, resources: &mut IdentifiedResources) -> Result<()> {
        println!("Splitting PBF into tiles at {:?}", self.out_dir);
        println!("  Pass 3 (Split): Loading and distributing nodes...");

        let mut node_locs: AHashMap<i64, (f64, f64)> =
            AHashMap::with_capacity(resources.needed_nodes.len());
        // Flags are sparse (stations, turn cycles and restriction via nodes), so
        // keep them out of the already-large node location value.
        let mut flagged_nodes: AHashMap<i64, u8> = AHashMap::new();
        let mut tile_buffers: AHashMap<TileCoord, Vec<u8>> = AHashMap::new();

        {
            let f = std::fs::File::open(&self.osm_path)?;
            let mut pbf = OsmPbfReader::new(f);
            let mut nodes_processed = 0u64;

            for obj in pbf.iter() {
                let obj = obj?;
                if let OsmObj::Node(n) = obj {
                    let nid = n.id.0;
                    if resources.needed_nodes.binary_search(&nid).is_ok() {
                        let lat = n.lat();
                        let lon = n.lon();
                        node_locs.insert(nid, (lon, lat));

                        let mut flags = node_flags(&n.tags);
                        if resources.restriction_via_nodes.contains(&nid) {
                            flags |= NODE_FLAG_RESTRICTION;
                        }
                        if flags != 0 {
                            flagged_nodes.insert(nid, flags);
                        }

                        let tile = TileCoord::from_point(lon, lat);
                        let item = TileItem::Node(NodeData {
                            id: nid,
                            lon,
                            lat,
                            flags,
                        });

                        let bytes = bincode::serialize(&item)?;
                        let len = bytes.len() as u32;
                        let entry = tile_buffers
                            .entry(tile)
                            .or_insert_with(|| Vec::with_capacity(64 * 1024));
                        entry.extend_from_slice(&len.to_le_bytes());
                        entry.extend_from_slice(&bytes);

                        nodes_processed += 1;
                        if entry.len() > MAX_TILE_BUFFER_SIZE {
                            let data = std::mem::replace(entry, Vec::with_capacity(64 * 1024));
                            self.append_to_tile_file(tile, &data)?;
                        }
                    }
                }
            }
            println!("  Processed {} nodes.", nodes_processed);
        }

        println!(
            "  Loaded {} node locations. Flushing node buffers...",
            node_locs.len()
        );
        self.flush_buffers(&mut tile_buffers)?;
        tile_buffers.clear();

        // These IDs are only needed for the node pass. Release them before the
        // node-location map and way pass overlap in memory. Restriction-via
        // information is already encoded in NodeData/flagged_nodes.
        resources.needed_nodes.clear();
        resources.needed_nodes.shrink_to_fit();
        resources.restriction_via_nodes.clear();
        resources.restriction_via_nodes.shrink_to_fit();

        println!("  Pass 4 (Split): Distributing ways...");
        {
            let f = std::fs::File::open(&self.osm_path)?;
            let mut pbf = OsmPbfReader::new(f);
            let mut ways_processed = 0u64;

            for obj in pbf.iter() {
                let obj = obj?;
                if let OsmObj::Way(w) = obj {
                    let wid = w.id.0;
                    // The resource pass already applied the upstream-style
                    // mode filter and relation inheritance. Do not re-admit
                    // unrelated rail/road ways just because they share a node
                    // with a bus tile.
                    if resources.kept_ways.binary_search(&wid).is_err() {
                        continue;
                    }
                    if !OsmBuilder::is_valid_way(&w) {
                        continue;
                    }

                    let mut way_tiles = AHashSet::new();
                    for nid in &w.nodes {
                        if let Some(&(lon, lat)) = node_locs.get(&nid.0) {
                            way_tiles.insert(TileCoord::from_point(lon, lat));
                        }
                    }
                    if way_tiles.is_empty() {
                        continue;
                    }

                    // These are pfaedle routing levels/one-way rules, not the OSM
                    // physical `level=*` tag. collapseEdges compares exactly these
                    // properties in upstream C++.
                    let item = TileItem::Way(WayData {
                        id: wid,
                        refs: w.nodes.iter().map(|node| node.0).collect(),
                        bus_level: bus_level(&w.tags),
                        coach_level: coach_level(&w.tags),
                        oneway: bus_oneway(&w.tags),
                        restriction: resources.restricted_ways.contains(&wid),
                    });
                    let bytes = bincode::serialize(&item)?;
                    let len = bytes.len() as u32;

                    for tile in way_tiles {
                        let entry = tile_buffers
                            .entry(tile)
                            .or_insert_with(|| Vec::with_capacity(64 * 1024));
                        entry.extend_from_slice(&len.to_le_bytes());
                        entry.extend_from_slice(&bytes);

                        // A way crossing a tile boundary needs its out-of-tile
                        // endpoint nodes. Carry the same collapse-protection flags
                        // onto these ghost copies.
                        for nid in &w.nodes {
                            if let Some(&(lon, lat)) = node_locs.get(&nid.0) {
                                let node_tile = TileCoord::from_point(lon, lat);
                                if node_tile != tile {
                                    let node_item = TileItem::Node(NodeData {
                                        id: nid.0,
                                        lon,
                                        lat,
                                        flags: flagged_nodes.get(&nid.0).copied().unwrap_or(0),
                                    });
                                    let node_bytes = bincode::serialize(&node_item)?;
                                    let node_len = node_bytes.len() as u32;
                                    entry.extend_from_slice(&node_len.to_le_bytes());
                                    entry.extend_from_slice(&node_bytes);
                                }
                            }
                        }

                        if entry.len() > MAX_TILE_BUFFER_SIZE {
                            let data = std::mem::replace(entry, Vec::with_capacity(64 * 1024));
                            self.append_to_tile_file(tile, &data)?;
                        }
                    }

                    ways_processed += 1;
                    if ways_processed % 500_000 == 0 {
                        println!("    Processed {} ways...", ways_processed);
                    }
                }
            }
            println!("  Processed {} total ways.", ways_processed);
        }

        println!("  Flushing way buffers...");
        self.flush_buffers(&mut tile_buffers)?;
        Ok(())
    }

    fn append_to_tile_file(&self, tile: TileCoord, data: &[u8]) -> Result<()> {
        if data.is_empty() {
            return Ok(());
        }
        let path = self.out_dir.join(format!("tile_{}_{}.bin", tile.x, tile.y));
        let mut file = OpenOptions::new().create(true).append(true).open(&path)?;
        file.write_all(data)?;
        Ok(())
    }

    fn flush_buffers(&self, buffers: &mut AHashMap<TileCoord, Vec<u8>>) -> Result<()> {
        for (tile, data) in buffers.drain() {
            self.append_to_tile_file(tile, &data)?;
        }
        Ok(())
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub enum TileItem {
    Node(NodeData),
    Way(WayData),
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct NodeData {
    pub id: i64,
    pub lon: f64,
    pub lat: f64,
    pub flags: u8,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct WayData {
    pub id: i64,
    pub refs: Vec<i64>,
    /// Upstream pfaedle `[bus, coach]` routing/filter level.
    pub bus_level: u8,
    /// Upstream pfaedle `[coach]` override routing/filter level.
    pub coach_level: u8,
    /// 0=bidirectional, 1=forward only, 2=reverse only.
    pub oneway: u8,
    /// This way participates in a restriction relation. Only segments adjacent
    /// to a protected via node are marked restricted when the graph is built.
    pub restriction: bool,
}
