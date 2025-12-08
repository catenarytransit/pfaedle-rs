mod graph;
mod gtfs_load;
mod matcher;
mod osm_load;
mod pathfinding;
mod router;

use anyhow::{Context, Result};
use clap::Parser;
use std::path::PathBuf;

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
    #[arg(short, long)]
    out_dir: Option<PathBuf>,

    /// Wipe existing shapes.txt
    #[arg(short, long, default_value_t = false)]
    wipe_shapes: bool,
}

#[derive(serde::Serialize)]
struct ShapeRecord {
    shape_id: String,
    shape_pt_lat: f64,
    shape_pt_lon: f64,
    shape_pt_sequence: usize,
}

fn main() -> Result<()> {
    let args = Args::parse();
    println!("Running pfaedle-rs with args: {:?}", args);

    let out_dir = args.out_dir.as_ref().unwrap_or(&args.gtfs_dir);
    let shapes_path = out_dir.join("shapes.txt");

    if args.wipe_shapes {
        if shapes_path.exists() {
            println!("Wiping existing shapes.txt at {:?}", shapes_path);
            std::fs::remove_file(&shapes_path)?;
        }
    }

    // 1. Load GTFS
    let gtfs_data = gtfs_load::load_gtfs(&args.gtfs_dir)?;

    // 2. Load OSM
    let osm_data = osm_load::load_osm(&args.osm_file, &gtfs_data.used_route_types)?;

    // 3. Match
    let results = matcher::match_patterns(&gtfs_data, &osm_data);

    // 4. Write shapes.txt
    println!("Writing shapes.txt to {:?}", shapes_path);
    let mut wtr = csv::Writer::from_path(&shapes_path)?;

    for shape_res in results.values() {
        for (i, (lat, lon)) in shape_res.points.iter().enumerate() {
            wtr.serialize(ShapeRecord {
                shape_id: shape_res.shape_id.clone(),
                shape_pt_lat: *lat,
                shape_pt_lon: *lon,
                shape_pt_sequence: i + 1,
            })?;
        }
    }
    wtr.flush()?;

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
    let mut trip_to_shape = std::collections::HashMap::new();
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

    // Replace original trips.txt
    std::fs::rename(trips_out_path, trips_path)?;

    println!("Done!");

    Ok(())
}
