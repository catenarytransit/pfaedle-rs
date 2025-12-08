mod color;
mod graph;
mod gtfs_load;
mod matcher;
mod mots;
mod osm_load;
mod pathfinding;
mod router;

use anyhow::{Context, Result};
use clap::Parser;
use gtfs_structures::RouteType;
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

    /// MOTs to calculate shapes for, comma sep.
    #[arg(short, long, default_value = "all")]
    mots: String,

    /// Write colours to routes.txt
    #[arg(long, visible_alias = "write-colors", default_value_t = false)]
    write_colours: bool,
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

    // 0. Parse MOTS
    let allowed_mots = mots::get_categories_from_string(&args.mots)
        .map_err(|e| anyhow::anyhow!("Invalid MOTs string: {}", e))?;

    println!("Allowed MOTS: {:?}", allowed_mots);

    // 1. Load GTFS
    let gtfs_data = gtfs_load::load_gtfs(&args.gtfs_dir, &allowed_mots)?;

    // 2. Load OSM
    // Calculate BBox from GTFS stops
    let bbox = if gtfs_data.gtfs.stops.is_empty() {
        None
    } else {
        let mut min_lat = f64::MAX;
        let mut max_lat = f64::MIN;
        let mut min_lon = f64::MAX;
        let mut max_lon = f64::MIN;

        for stop in gtfs_data.gtfs.stops.values() {
            min_lat = min_lat.min(stop.latitude.expect("Stop missing latitude"));
            max_lat = max_lat.max(stop.latitude.expect("Stop missing latitude"));
            min_lon = min_lon.min(stop.longitude.expect("Stop missing longitude"));
            max_lon = max_lon.max(stop.longitude.expect("Stop missing longitude"));
        }

        let has_rail = gtfs_data.used_route_types.contains(&RouteType::Rail);
        let padding_km = if has_rail { 200.0 } else { 100.0 };
        println!(
            "Stops BBox: lat [{:.4}, {:.4}], lon [{:.4}, {:.4}]. Padding: {} km",
            min_lat, max_lat, min_lon, max_lon, padding_km
        );

        let lat_padding = padding_km / 111.132;
        let center_lat = (min_lat + max_lat) / 2.0;
        let cos_lat = center_lat.to_radians().cos().abs().max(0.1);
        let lon_padding = padding_km / (111.132 * cos_lat);

        let final_bbox = (
            min_lon - lon_padding,
            min_lat - lat_padding,
            max_lon + lon_padding,
            max_lat + lat_padding,
        );
        println!(
            "Cropping OSM usage to BBox: lat [{:.4}, {:.4}], lon [{:.4}, {:.4}]",
            final_bbox.1, final_bbox.3, final_bbox.0, final_bbox.2
        );
        Some(final_bbox)
    };

    let osm_data = osm_load::load_osm(&args.osm_file, &gtfs_data.used_route_types, bbox)?;

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

    if args.write_colours {
        println!("Updating routes.txt with colors...");
        // 1. Build map route_id -> (bg_color, fg_color)
        let mut route_colors = std::collections::HashMap::new();
        // Iterate over results: stop_pattern -> shape_result
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
                                // Insert if not present. Maybe overwrite?
                                // If multiple patterns map to same route but different colors, we have ambiguity.
                                // First one wins for now.
                                route_colors.entry(route_id.clone()).or_insert((bg, fg));
                            }
                        }
                    }
                }
            }
        }

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
        std::fs::rename(routes_out_path, routes_path)?;
    }

    println!("Done!");

    Ok(())
}
