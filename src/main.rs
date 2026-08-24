mod color;
#[path = "trgraph.rs"]
mod graph;
mod gtfs_load;
mod hilbert;
mod matcher;
mod mots;
mod osm_filter;
#[path = "osm_builder.rs"]
mod osm_load;
#[path = "osm_split_upstream.rs"]
mod osm_split;
mod pathfinding;
mod router;
#[path = "streaming_bus.rs"]
mod streaming_matcher;
mod tile_loader;
mod upstream_graph;
mod upstream_match;

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

    /// Number of concurrent full-graph matching workers
    #[arg(long)]
    match_threads: Option<usize>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    println!("Running pfaedle-rs with args: {:?}", args);

    #[cfg(not(windows))]
    if args.low_priority {
        if let Err(e) = rustix::process::nice(10) {
            eprintln!("Failed to set priority: {}", e);
        } else {
            println!("Process priority set to low (nice +10).");
        }
    }

    #[cfg(windows)]
    if args.low_priority {
        eprintln!("Warning: --low-priority is not supported on Windows.");
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
    }

    #[derive(serde::Serialize)]
    struct ShapeRecordOut<'a> {
        shape_id: &'a str,
        shape_pt_lat: f64,
        shape_pt_lon: f64,
        shape_pt_sequence: usize,
    }

    // 2. Match Patterns (loads OSM on demand)
    // We strictly skip processing small roads for performance unless configured otherwise
    let cache_file_path = out_dir.join("shapes_cache.bin");
    let results = matcher::match_patterns(
        &gtfs_data,
        &args.osm_file,
        args.skip_small_roads,
        &cache_file_path,
        args.match_threads,
    )?;

    // 4. Write shapes.txt
    println!("Updating shapes.txt at {:?}", shapes_path);

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
    }

    let will_wipe_all = args.mots == "all" && args.wipe_shapes;
    let shapes_new_path = out_dir.join("shapes-new.txt");

    let mut wtr = csv::Writer::from_path(&shapes_new_path)?;

    let mut new_shape_ids = ahash::AHashSet::new();
    for res in results.values() {
        if !res.empty_geometry {
            new_shape_ids.insert(res.shape_id.clone());
        }
    }

    if shapes_path.exists() && !will_wipe_all {
        println!("Streaming existing shapes (optimized)...");
        match csv::Reader::from_path(&shapes_path) {
            Ok(mut rdr) => {
                for result in rdr.deserialize() {
                    let record: RawShape = match result {
                        Ok(r) => r,
                        Err(e) => {
                            eprintln!("Warning: Error reading shape record: {}", e);
                            continue;
                        }
                    };
                    // Keep the old shape only if it's not explicitly replaced by MOT selection AND not overwritten by a newly matched shape.
                    if !shapes_to_replace.contains(&record.id)
                        && !new_shape_ids.contains(&record.id)
                    {
                        wtr.serialize(ShapeRecordOut {
                            shape_id: &record.id,
                            shape_pt_lat: record.latitude,
                            shape_pt_lon: record.longitude,
                            shape_pt_sequence: record.sequence,
                        })?;
                    }
                }
            }
            Err(e) => {
                eprintln!(
                    "Warning: Failed to read existing shapes: {}. Proceeding with empty set.",
                    e
                );
            }
        }
    }

    // Insert NEW shapes
    println!("Appending new shapes from cache...");
    let mut new_shapes_count = 0;
    if cache_file_path.exists() {
        let cache_file = File::open(&cache_file_path)?;
        let mut cache_reader = BufReader::new(cache_file);
        while let Ok(rec) =
            bincode::deserialize_from::<_, crate::matcher::BinaryShapeRecord>(&mut cache_reader)
        {
            wtr.serialize(ShapeRecordOut {
                shape_id: &rec.shape_id,
                shape_pt_lat: rec.shape_pt_lat,
                shape_pt_lon: rec.shape_pt_lon,
                shape_pt_sequence: rec.shape_pt_sequence,
            })?;
            new_shapes_count += 1;
        }

        std::fs::remove_file(&cache_file_path).ok();
    }
    println!("Total new shape points written: {}", new_shapes_count);

    wtr.flush()?;
    drop(wtr);

    std::fs::rename(&shapes_new_path, &shapes_path)?;

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
