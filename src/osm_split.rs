use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader};
use rayon::prelude::*;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use crate::osm_load::{OsmBuilder, PreRelation};
use crate::tile::TileCoord;

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

    /// Split the OSM file into tiles based on the identified resources.
    /// This performs a SINGLE pass (Pass 3 & 4 combined effectively) over the PBF
    /// and writes binary buckets for each tile.
    pub fn split_pbf(
        &self,
        needed_nodes: &[i64], // Sorted
        ways_in_relations: &AHashSet<i64>,
        ways_in_ferry_relations: &AHashSet<i64>,
    ) -> Result<()> {
        println!("Splitting PBF into tiles at {:?}", self.out_dir);

        // We need to write to many files. Keeping them all open might hit handle limits.
        // Instead, we can buffer in memory per tile and write chunks, or use a limited LRU of open files.
        // For simplicity: Buffer per tile in memory (Vec<u8> using bincode) and flush periodically?
        // Or better: Just collect all Nodes/Ways, then partition?
        // 27M nodes * 20 bytes = 540MB. Feasible to load all needed nodes locations into memory?
        // Yes. Let's load all needed nodes into a map: NodeId -> (Lon, Lat).
        // Then when we scan Ways, we can look up their nodes to determine tile.
        // BUT, we also need to write the Nodes themselves to the tile files.

        // Strategy:
        // 1. Scan Nodes. If needed, store in memory map `NodeLocs` AND write to `tile_X_Y.nodes.bin`.
        // 2. Scan Ways. If interesting, look up nodes in `NodeLocs`. Determine which tiles the way intersects.
        //    Write way to `tile_X_Y.ways.bin` for ALL intersected tiles.

        println!("  Pass 3 (Split): Loading and distributing nodes...");

        // We'll use a thread-safe map of BufWriters protected by Mutex?
        // Or just channels?
        // Let's use a DashMap of Mutex<BufWriter>?
        // Or simple: AHashMap<TileCoord, Vec<u8>> and flush when large.

        let tile_buffers = std::sync::Arc::new(dashmap::DashMap::new());

        // Helper to write generic item to tile buffer
        let write_to_tile = |tile: TileCoord, data: &[u8]| {
            let mut entry = tile_buffers
                .entry(tile)
                .or_insert_with(|| Vec::with_capacity(1024 * 1024));
            entry.extend_from_slice(data);
        };

        // 1. Nodes
        // We also build a NodeLoc map to help place Ways later.
        // Only needed nodes are stored.
        // NodeLoc: id -> (lon, lat) (f32 is enough for bbox checks? No, need precision for output?
        // We only use this map to determine WHICH tiles a Way belongs to. Precision is key.
        let mut node_locs: AHashMap<i64, (f64, f64)> = AHashMap::with_capacity(needed_nodes.len());

        {
            let f = std::fs::File::open(&self.osm_path)?;
            let mut pbf = OsmPbfReader::new(f);

            for obj in pbf.iter() {
                let obj = obj?;
                if let OsmObj::Node(n) = obj {
                    let nid = n.id.0;
                    if needed_nodes.binary_search(&nid).is_ok() {
                        let lat = n.lat();
                        let lon = n.lon();
                        node_locs.insert(nid, (lon, lat));

                        // Determine tile
                        let tile = TileCoord::from_point(lon, lat);

                        // Serialize Node (we use a custom simple binary format or bincode)
                        // Bincode is easy.
                        // We wrap it in a custom "TileItem" enum?
                        // Or separate files for nodes/ways?
                        // Let's use `TileItem` enum.
                        let item = TileItem::Node(NodeData {
                            id: nid,
                            lon,
                            lat,
                            tags: n
                                .tags
                                .iter()
                                .map(|(k, v)| (k.clone().into(), v.clone().into()))
                                .collect(),
                        });

                        let bytes = bincode::serialize(&item)?;
                        // Write size prefix? Or bincode handles stream?
                        // Bincode size-limit is good, but usually we need framing.
                        // Let's ensure we can read it back. Framed stream.
                        // u32 length + bytes.
                        let len = bytes.len() as u32;
                        let mut entry = tile_buffers
                            .entry(tile)
                            .or_insert_with(|| Vec::with_capacity(64 * 1024));
                        entry.extend_from_slice(&len.to_le_bytes());
                        entry.extend_from_slice(&bytes);
                    }
                }
            }
        }
        println!(
            "  Loaded {} nodes locations. Flushing node buffers...",
            node_locs.len()
        );
        self.flush_buffers(&tile_buffers, "bin")?;
        tile_buffers.clear();

        // 2. Ways
        println!("  Pass 4 (Split): Distributing ways...");
        {
            let f = std::fs::File::open(&self.osm_path)?;
            let mut pbf = OsmPbfReader::new(f);

            for obj in pbf.iter() {
                let obj = obj?;
                if let OsmObj::Way(w) = obj {
                    let wid = w.id.0;

                    // Logic from osm_load: is_infra, is_rel_member
                    // We need to replicate that logic or pass in the sets.
                    // We passed in `ways_in_relations` etc.

                    let is_infra =
                        OsmBuilder::is_infrastructure(&w) || ways_in_ferry_relations.contains(&wid);
                    let is_platform = OsmBuilder::is_platform(&w);
                    let is_rel_member = ways_in_relations.contains(&wid) && !is_platform;

                    if !is_infra && !is_rel_member {
                        continue;
                    }
                    if !OsmBuilder::is_valid_way(&w) {
                        continue;
                    }

                    // Determine tiles for this way
                    let mut ways_tiles = AHashSet::new();

                    // We look at all nodes to find bbox/tiles
                    // AND strictly include every tile that contains a node of this way.
                    // Plus we might want to "connect" them if they span tiles (corridor).
                    // But TileCache::get_for_segment logic handles loading intermediate tiles.
                    // So just adding to tiles containing nodes is usually sufficient *if* segments are short.
                    // OSM ways usually are short.
                    // If a way is long, it has intermediate nodes.

                    for nid in &w.nodes {
                        if let Some(&(lon, lat)) = node_locs.get(&nid.0) {
                            let tile = TileCoord::from_point(lon, lat);
                            ways_tiles.insert(tile);
                        }
                    }

                    if ways_tiles.is_empty() {
                        // Should not happen if we kept all needed nodes
                        continue;
                    }

                    let item = TileItem::Way(WayData {
                        id: wid,
                        tags: w
                            .tags
                            .iter()
                            .map(|(k, v)| (k.clone().into(), v.clone().into()))
                            .collect(),
                        refs: w.nodes.iter().map(|n| n.0).collect(),
                    });
                    let bytes = bincode::serialize(&item)?;
                    let len = bytes.len() as u32;

                    for tile in ways_tiles {
                        let mut entry = tile_buffers
                            .entry(tile)
                            .or_insert_with(|| Vec::with_capacity(64 * 1024));
                        entry.extend_from_slice(&len.to_le_bytes());
                        entry.extend_from_slice(&bytes);
                    }
                }
            }
        }

        println!("  Flushing way buffers...");
        self.flush_buffers(&tile_buffers, "bin")?;

        Ok(())
    }

    fn flush_buffers(
        &self,
        buffers: &dashmap::DashMap<TileCoord, Vec<u8>>,
        ext: &str,
    ) -> Result<()> {
        buffers
            .into_iter()
            .par_bridge()
            .try_for_each(|entry| -> Result<()> {
                let (tile, data) = entry.pair();
                if data.is_empty() {
                    return Ok(());
                }

                let path = self
                    .out_dir
                    .join(format!("tile_{}_{}.{}", tile.x, tile.y, ext));
                // Append mode
                let mut file = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&path)?;
                file.write_all(data)?;
                Ok(())
            })?;
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
    pub tags: Vec<(String, String)>,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct WayData {
    pub id: i64,
    pub tags: Vec<(String, String)>,
    pub refs: Vec<i64>,
}
