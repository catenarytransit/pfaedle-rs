mod color;
mod graph;
mod gtfs_load;
mod hilbert;
mod matcher;
mod mots;
mod osm_load;
mod osm_split;
mod pathfinding;
mod router;
mod segment_matcher;
mod streaming_matcher;
mod tile;

use ahash::AHashMap;
use anyhow::{Context, Result};
use clap::Parser;
use geo::Point;
use gtfs_structures::RouteType;
use serde::Deserialize;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

#[cfg(not(target_env = "msvc"))]
use tikv_jemallocator::Jemalloc;

#[cfg(not(target_env = "msvc"))]
#[global_allocator]
static GLOBAL: Jemalloc = Jemalloc;

fn normalize_csv(path: &Path) -> Result<()> {
    if !path.exists() {
        return Ok(());
    }
    println!("Normalizing CSV at {:?}", path);

    // Read with flexible whitespace trimming
    let mut rdr = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .flexible(true)
        .from_path(path)?;

    // We collect all records first to avoid reading/writing same file issues
    // (though creating a temp file is safer)
    let headers = rdr.headers()?.clone();
    let mut records = Vec::new();
    for result in rdr.records() {
        records.push(result?);
    }
    drop(rdr);

    // Write back
    let mut wtr = csv::Writer::from_path(path)?;
    wtr.write_record(&headers)?;
    for record in records {
        wtr.write_record(&record)?;
    }
    wtr.flush()?;

    Ok(())
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to GTFS directory or zip
    #[arg(short, long)]
    gtfs_dir: PathBuf,

    /// Path to OSM PBF file
    #[arg(short, long)]
    osm_file: PathBuf,

    /// Output directory for shapes.txt (defaults to gtfs_dir)
    #[arg(long)]
    out_dir: Option<PathBuf>,

    /// Wipe existing shapes.txt
    #[arg(short, long, default_value_t = false)]
    wipe_shapes: bool,

    /// Skip importing residential/service roads if they are not part of a route relation
    #[arg(long, default_value_t = false)]
    skip_small_roads: bool,

    /// MOTs to calculate shapes for, comma sep.
    #[arg(short, long, default_value = "all")]
    mots: String,

    /// Write colours to routes.txt
    #[arg(long, visible_alias = "write-colors", default_value_t = false)]
    write_colours: bool,

    /// Run with low priority (nice 10)
    #[arg(long, default_value_t = false)]
    low_priority: bool,
}

#[derive(Debug, Clone)]
pub struct ShapePoint {
    pub geometry: Point<f64>,
    pub sequence: usize,
}

#[derive(Deserialize)]
struct RawShape {
    #[serde(rename = "shape_id")]
    pub id: String,
    #[serde(rename = "shape_pt_lat")]
    pub latitude: f64,
    #[serde(rename = "shape_pt_lon")]
    pub longitude: f64,
    #[serde(rename = "shape_pt_sequence")]
    pub sequence: usize,
    // #[serde(rename = "shape_dist_traveled")]
    // pub dist_traveled: Option<f32>,
}

#[derive(serde::Serialize)]
struct ShapeRecordOut<'a> {
    shape_id: &'a str,
    shape_pt_lat: f64,
    shape_pt_lon: f64,
    shape_pt_sequence: usize,
}

/// O(n) check; only sort if any sequence is out of order.
fn maybe_sort_by_sequence(points: &mut Vec<ShapePoint>) {
    if points.len() < 2 {
        return;
    }

    let mut prev = points[0].sequence;
    let mut sorted = true;

    for p in points.iter().skip(1) {
        if p.sequence < prev {
            sorted = false;
            break;
        }
        prev = p.sequence;
    }

    if !sorted {
        // In-place, no extra allocation
        points.sort_unstable_by_key(|p| p.sequence);
    }
}

/// Reads a shapes.txt file and aggregates points into a HashMap.
///
/// This implementation assumes the input CSV is sorted by `shape_id` for efficiency,
/// but will handle unsorted data correctly (just less efficiently due to lookups/resizing).
/// ACTUALLY, the streaming logic provided by user *relies* on sorting to flush.
/// If not sorted, we'd overwrite or need to append.
/// Most GTFS shapes.txt are sorted. If not, this logic effectively splits them if they are interleaved
/// (which is fine, we just get multiple entries potentially? No, HashMap overwrites).
/// NO, the user logic accumulates `current_points` and inserts once ID changes.
/// If IDs are interleaved (A, B, A), we would insert A, then B, then overwrite A with the second chunk.
/// This would be DATA LOSS for interleaved files.
/// To be SAFE, we should probably check if key exists and append?
/// OR, just assume standard GTFS which is grouped. usage of `sort` command on linux can ensure this
/// but we are in rust.
/// Let's assume standard grouping.
pub fn faster_shape_reader(
    path: PathBuf,
) -> Result<AHashMap<String, Vec<ShapePoint>>, Box<dyn Error>> {
    let file = File::open(path).context("Failed to open shapes.txt")?;
    let buf_reader = BufReader::new(file);
    let mut rdr = csv::Reader::from_reader(buf_reader);

    // Initial capacity guess
    let mut shapes: AHashMap<String, Vec<ShapePoint>> = AHashMap::with_capacity(1000);

    // State buffers
    let mut current_points: Vec<ShapePoint> = Vec::with_capacity(500);
    let mut current_shape_id: Option<String> = None;

    for result in rdr.deserialize() {
        let record: RawShape = result?;

        if let Some(ref curr_id) = current_shape_id {
            if *curr_id != record.id {
                // ID changed
                if !current_points.is_empty() {
                    maybe_sort_by_sequence(&mut current_points);
                    // If key exists (interleaved), we append?
                    // Safe approach: entry().or_default().extend(...)
                    shapes
                        .entry(curr_id.clone())
                        .or_default()
                        .append(&mut current_points); // Moves contents

                    // current_points is now empty but capacity kept?
                    // append moves elements. Capacity of `current_points` remains?
                    // No, `append` drains other. `current_points` becomes empty.
                    // We might need to re-reserve if capacity dropped? Usually Vec keeps capacity.
                }
                current_shape_id = Some(record.id.clone());
            }
        } else {
            current_shape_id = Some(record.id.clone());
        }

        let point = ShapePoint {
            geometry: Point::new(record.longitude, record.latitude),
            sequence: record.sequence,
        };
        current_points.push(point);
    }

    if let Some(last_id) = current_shape_id {
        if !current_points.is_empty() {
            maybe_sort_by_sequence(&mut current_points);
            shapes
                .entry(last_id)
                .or_default()
                .append(&mut current_points);
        }
    }

    Ok(shapes)
}

fn main() -> Result<()> {
    let args = Args::parse();
    println!("Running pfaedle-rs with args: {:?}", args);

    if args.low_priority {
        if let Err(e) = rustix::process::nice(10) {
            eprintln!("Failed to set priority: {}", e);
        } else {
            println!("Process priority set to low (nice +10).");
        }
    }

    let out_dir = args.out_dir.as_ref().unwrap_or(&args.gtfs_dir);
    let shapes_path = out_dir.join("shapes.txt");

    // 0. Parse MOTS
    let allowed_mots = mots::get_categories_from_string(&args.mots)
        .map_err(|e| anyhow::anyhow!("Invalid MOTs string: {}", e))?;

    println!("Allowed MOTS: {:?}", allowed_mots);

    // Normalise trips.txt and routes.txt to handle spaces
    normalize_csv(&args.gtfs_dir.join("trips.txt"))?;
    normalize_csv(&args.gtfs_dir.join("routes.txt"))?;

    // 1. Load GTFS
    let gtfs_data = gtfs_load::load_gtfs(&args.gtfs_dir, &allowed_mots)
        .context("Failed to load GTFS for Pfaedle")?;

    // 2. Match Patterns (loads OSM on demand)
    // We strictly skip processing small roads for performance unless configured otherwise
    let results = matcher::match_patterns(&gtfs_data, &args.osm_file, args.skip_small_roads)?;

    // 4. Write shapes.txt
    println!("Updating shapes.txt at {:?}", shapes_path);

    let mut all_shapes: AHashMap<String, Vec<ShapePoint>> = AHashMap::new();

    if shapes_path.exists() {
        // Optimization: If we are going to wipe all shapes anyway, don't read them!
        let will_wipe_all = args.mots == "all" && args.wipe_shapes;

        if !will_wipe_all {
            println!("Reading existing shapes (optimized)...");
            // Use our new faster reader
            match faster_shape_reader(shapes_path.clone()) {
                Ok(shapes) => {
                    all_shapes = shapes;
                    println!("Loaded {} existing shapes.", all_shapes.len());
                }
                Err(e) => {
                    eprintln!(
                        "Warning: Failed to read existing shapes: {}. Proceeding with empty set.",
                        e
                    );
                }
            }
        } else {
            println!("Skipping read of existing shapes because wipe_shapes is set and MOTs=all.");
        }
    }

    // Determine shapes to wipe/replace based on selected MOTs
    let mut shapes_to_replace = ahash::AHashSet::new();
    if args.mots != "all" {
        for (_trip_id, trip) in &gtfs_data.gtfs.trips {
            let route = gtfs_data.gtfs.routes.get(&trip.route_id);
            if let Some(r) = route {
                let cat = mots::map_route_type_to_category(r.route_type);
                if allowed_mots.contains(&cat) {
                    // Treat empty shape_id as null/missing
                    if let Some(sid) = &trip.shape_id {
                        if !sid.is_empty() {
                            shapes_to_replace.insert(sid.clone());
                        }
                    }
                }
            }
        }
        println!(
            "Identified {} shape_ids for selected MOTs.",
            shapes_to_replace.len()
        );

        // When wipe_shapes is set, remove these shapes (even before computing new ones)
        if args.wipe_shapes && !shapes_to_replace.is_empty() {
            let before_count = all_shapes.len();
            all_shapes.retain(|id, _| !shapes_to_replace.contains(id));
            println!(
                "Wiped {} shapes for selected MOTs (kept {} other shapes).",
                before_count - all_shapes.len(),
                all_shapes.len()
            );
        }
    } else {
        if args.wipe_shapes {
            println!("MOTs='all' and wipe_shapes=true. Clearing all existing shapes.");
            all_shapes.clear();
        }
    }

    // Remove obsolete shapes (only if not already wiped above)
    if !args.wipe_shapes && !shapes_to_replace.is_empty() {
        let before_count = all_shapes.len();
        all_shapes.retain(|id, _| !shapes_to_replace.contains(id));
        println!(
            "Pruned {} existing shapes (kept {}).",
            before_count - all_shapes.len(),
            all_shapes.len()
        );
    }

    // Insert NEW shapes
    for shape_res in results.values() {
        let mut points = Vec::with_capacity(shape_res.points.len());
        for (i, (lat, lon)) in shape_res.points.iter().enumerate() {
            points.push(ShapePoint {
                geometry: Point::new(*lon, *lat),
                sequence: i + 1,
            });
        }
        all_shapes.insert(shape_res.shape_id.clone(), points);
    }

    // Write everything back
    // We should probably sort by shape_id to be nice?
    // And for each shape, ensure points valid?
    let mut wtr = csv::Writer::from_path(&shapes_path)?;

    // Convert HashMap to Vec for sorting keys (optional but good practice for consistency)
    let mut sorted_keys: Vec<&String> = all_shapes.keys().collect();
    sorted_keys.sort();

    for key in sorted_keys {
        if let Some(points) = all_shapes.get(key) {
            for point in points {
                wtr.serialize(ShapeRecordOut {
                    shape_id: key,
                    shape_pt_lat: point.geometry.y(),
                    shape_pt_lon: point.geometry.x(),
                    shape_pt_sequence: point.sequence,
                })?;
            }
        }
    }

    wtr.flush()?;
    drop(wtr);

    // 5. Update trips.txt
    // We need to associate trips with new shape_ids
    // gtfs-structures gives us trips, but we need to write the file back.
    // We will read the original trips.txt as raw CSV and update it.
    let trips_path = args.gtfs_dir.join("trips.txt");
    let trips_out_path = out_dir.join("trips-new.txt"); // Write to new file first then rename?
    // Or just overwrite if in-place.
    // But `gtfs-structures` might have locked it? No, it loads into memory.

    println!("Updating trips.txt...");

    // Create a mapping from trip_id to shape_id
    let mut trip_to_shape = ahash::AHashMap::new();
    for (pattern, trip_ids) in &gtfs_data.patterns {
        if let Some(res) = results.get(pattern) {
            for t_id in trip_ids {
                trip_to_shape.insert(t_id.clone(), res.shape_id.clone());
            }
        }
    }

    // Read/Write trips.txt using csv crate generic records to preserve other columns
    let mut rdr = csv::Reader::from_path(&trips_path)?;
    let headers = rdr.headers()?.clone();

    // Check if shape_id exists in headers
    let shape_id_idx = headers.iter().position(|h| h == "shape_id");

    // Need to construct new headers if shape_id missing
    let mut new_headers = headers.clone();
    if shape_id_idx.is_none() {
        new_headers.push_field("shape_id");
    }

    // We'll write to a temporary file
    let mut wtr_trips = csv::Writer::from_path(&trips_out_path)?;
    wtr_trips.write_record(&new_headers)?;

    for result in rdr.records() {
        let record = result?;
        // Identify trip_id column
        // Assuming standard GTFS, but let's find it index dynamic
        let trip_id_col = headers
            .iter()
            .position(|h| h == "trip_id")
            .context("No trip_id column")?;
        let trip_id = &record[trip_id_col];

        let new_shape_id = trip_to_shape.get(trip_id);

        let mut new_record = csv::StringRecord::new();

        for (i, field) in record.iter().enumerate() {
            if Some(i) == shape_id_idx {
                // Update existing shape_id
                if let Some(sid) = new_shape_id {
                    new_record.push_field(sid);
                } else {
                    new_record.push_field(field); // Keep original if no match
                }
            } else {
                new_record.push_field(field);
            }
        }

        // If we added a column
        if shape_id_idx.is_none() {
            if let Some(sid) = new_shape_id {
                new_record.push_field(sid);
            } else {
                new_record.push_field("");
            }
        }

        wtr_trips.write_record(&new_record)?;
    }
    wtr_trips.flush()?;
    drop(wtr_trips);

    // Replace original trips.txt
    std::fs::rename(trips_out_path, trips_path)?;

    if args.write_colours {
        println!("Updating routes.txt with colors...");
        // 1. Build map route_id -> (bg_color, fg_color)
        let mut route_colors = ahash::AHashMap::new();

        // First, try to get colors from computed results
        for (pattern, shape_res) in &results {
            if let Some(raw_color) = &shape_res.matched_route_color {
                // Parse color
                if let Some((bg, fg)) = color::parse_color(raw_color) {
                    // Get trips for this pattern
                    if let Some(trip_ids) = gtfs_data.patterns.get(pattern) {
                        // Get route_id from first trip (all trips in pattern share route usually)
                        if let Some(first_trip_id) = trip_ids.first() {
                            if let Some(trip) = gtfs_data.gtfs.trips.get(first_trip_id) {
                                let route_id = &trip.route_id;
                                route_colors.entry(route_id.clone()).or_insert((bg, fg));
                            }
                        }
                    }
                }
            }
        }

        // If we haven't matched all routes, try to match colors from OSM relations
        // for any routes that don't have colors yet (handles case where shapes are already in place)
        // If we haven't matched all routes, try to match colors from OSM relations
        // for any routes that don't have colors yet (handles case where shapes are already in place)
        // NOTE: This fallback is currently disabled because we don't load full OSM data in main anymore.
        // If needed, we could use LightOsmData here, but we'd need to load it or get it from matcher.
        /*
        let routes_needing_colors: ahash::AHashSet<_> = gtfs_data
            .gtfs
            .routes
            .keys()
            .filter(|rid| !route_colors.contains_key(*rid))
            .cloned()
            .collect();

        if !routes_needing_colors.is_empty() {
            println!(
                "  Matching colors for {} routes without computed shapes...",
                routes_needing_colors.len()
            );

            // This requires osm_data which we don't have here anymore
        }
        */

        println!("  Found colors for {} routes.", route_colors.len());

        // 2. Read/Update routes.txt
        let routes_path = args.gtfs_dir.join("routes.txt");
        let routes_out_path = out_dir.join("routes-new.txt");
        let mut rdr = csv::Reader::from_path(&routes_path)?;
        let headers = rdr.headers()?.clone();

        // Check for existing color columns
        let color_idx = headers.iter().position(|h| h == "route_color");
        let text_color_idx = headers.iter().position(|h| h == "route_text_color");

        let mut new_headers = headers.clone();
        if color_idx.is_none() {
            new_headers.push_field("route_color");
        }
        if text_color_idx.is_none() {
            new_headers.push_field("route_text_color");
        }

        let mut wtr = csv::Writer::from_path(&routes_out_path)?;
        wtr.write_record(&new_headers)?;

        for result in rdr.records() {
            let record = result?;
            let route_id_col = headers
                .iter()
                .position(|h| h == "route_id")
                .context("No route_id in routes.txt")?;
            let route_id = &record[route_id_col];
            let colors = route_colors.get(route_id);

            let mut new_record = csv::StringRecord::new();

            // Reconstruct record
            for (i, field) in record.iter().enumerate() {
                if Some(i) == color_idx {
                    if let Some((bg, _)) = colors {
                        new_record.push_field(bg);
                    } else {
                        new_record.push_field(field);
                    }
                } else if Some(i) == text_color_idx {
                    if let Some((_, fg)) = colors {
                        new_record.push_field(fg);
                    } else {
                        new_record.push_field(field);
                    }
                } else {
                    new_record.push_field(field);
                }
            }

            // Append new columns if missing in original
            if color_idx.is_none() {
                if let Some((bg, _)) = colors {
                    new_record.push_field(bg);
                } else {
                    new_record.push_field("");
                }
            }
            if text_color_idx.is_none() {
                if let Some((_, fg)) = colors {
                    new_record.push_field(fg);
                } else {
                    new_record.push_field("");
                }
            }

            wtr.write_record(&new_record)?;
        }
        wtr.flush()?;
        drop(wtr);
        std::fs::rename(routes_out_path, routes_path)?;
    }

    println!("Done!");

    Ok(())
}
