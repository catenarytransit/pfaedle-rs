use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader};
use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use crate::osm_load::OsmBuilder;
use crate::tile::TileCoord;

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

        // Strategy:
        // 1. Scan Nodes. Store location in memory AND write to tile files.
        // 2. Scan Ways. Look up nodes to determine tiles. Write way to tile files.
        //
        // Key memory optimizations:
        // - Node tiles are flushed incrementally (every MAX_TILE_BUFFER_SIZE bytes).
        // - NodeData contains NO tags (they're never used downstream).
        // - WayData contains pre-parsed flags instead of raw tags.

        println!("  Pass 3 (Split): Loading and distributing nodes...");

        // Node location map: only stores (lon, lat) for tile assignment during way processing
        // 27M nodes * (8+8+8) bytes ≈ 650MB - acceptable
        let mut node_locs: AHashMap<i64, (f64, f64)> = AHashMap::with_capacity(needed_nodes.len());

        // Tile buffers with incremental flushing
        let mut tile_buffers: AHashMap<TileCoord, Vec<u8>> = AHashMap::new();

        {
            let f = std::fs::File::open(&self.osm_path)?;
            let mut pbf = OsmPbfReader::new(f);
            let mut nodes_processed = 0u64;

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

                        // Serialize Node (minimal: id + lon + lat, NO tags)
                        let item = TileItem::Node(NodeData { id: nid, lon, lat });

                        let bytes = bincode::serialize(&item)?;
                        let len = bytes.len() as u32;

                        let entry = tile_buffers
                            .entry(tile)
                            .or_insert_with(|| Vec::with_capacity(64 * 1024));
                        entry.extend_from_slice(&len.to_le_bytes());
                        entry.extend_from_slice(&bytes);

                        nodes_processed += 1;

                        // Flush large buffers incrementally
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

        // 2. Ways
        println!("  Pass 4 (Split): Distributing ways...");
        {
            let f = std::fs::File::open(&self.osm_path)?;
            let mut pbf = OsmPbfReader::new(f);
            let mut ways_processed = 0u64;

            for obj in pbf.iter() {
                let obj = obj?;
                if let OsmObj::Way(w) = obj {
                    let wid = w.id.0;

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

                    for nid in &w.nodes {
                        if let Some(&(lon, lat)) = node_locs.get(&nid.0) {
                            let tile = TileCoord::from_point(lon, lat);
                            ways_tiles.insert(tile);
                        }
                    }

                    if ways_tiles.is_empty() {
                        continue;
                    }

                    // Pre-parse tags into compact flags instead of storing all raw tag strings
                    let level = w
                        .tags
                        .get("level")
                        .and_then(|s| s.parse::<f32>().ok())
                        .unwrap_or(0.0) as i8;

                    let oneway = if w
                        .tags
                        .get("oneway")
                        .map_or(false, |s| s == "yes" || s == "true" || s == "1")
                    {
                        1u8
                    } else if w.tags.get("oneway").map_or(false, |s| s == "-1") {
                        2u8
                    } else {
                        0u8
                    };

                    // Check if way has railway tag (for mode flags)
                    let has_railway = w.tags.contains_key("railway");

                    let item = TileItem::Way(WayData {
                        id: wid,
                        refs: w.nodes.iter().map(|n| n.0).collect(),
                        level,
                        oneway,
                        has_railway,
                    });
                    let bytes = bincode::serialize(&item)?;
                    let len = bytes.len() as u32;

                    for tile in ways_tiles {
                        let entry = tile_buffers
                            .entry(tile)
                            .or_insert_with(|| Vec::with_capacity(64 * 1024));
                        entry.extend_from_slice(&len.to_le_bytes());
                        entry.extend_from_slice(&bytes);

                        // Flush large buffers incrementally
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

    /// Append data to a tile file.
    fn append_to_tile_file(&self, tile: TileCoord, data: &[u8]) -> Result<()> {
        if data.is_empty() {
            return Ok(());
        }
        let path = self.out_dir.join(format!("tile_{}_{}.bin", tile.x, tile.y));
        let mut file = OpenOptions::new().create(true).append(true).open(&path)?;
        file.write_all(data)?;
        Ok(())
    }

    /// Flush all remaining buffers to disk.
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

/// Minimal node data - only what's needed for tile graph construction.
/// Tags are NOT stored because they're never used downstream.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct NodeData {
    pub id: i64,
    pub lon: f64,
    pub lat: f64,
}

/// Way data with pre-parsed properties instead of raw tags.
/// This significantly reduces memory and disk usage.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct WayData {
    pub id: i64,
    pub refs: Vec<i64>,
    /// Level (floor) of the way, pre-parsed from tags.
    pub level: i8,
    /// Oneway flag: 0=bidirectional, 1=forward only, 2=reverse only
    pub oneway: u8,
    /// Whether this way has a railway tag (for mode classification).
    pub has_railway: bool,
}
