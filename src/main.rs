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

#[derive(serde::Serialize, serde::Deserialize)]
struct ShapeRecord {
    shape_id: String,
    shape_pt_lat: f64,
    shape_pt_lon: f64,
    shape_pt_sequence: usize,
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

    // 1. Load GTFS
    let gtfs_data = gtfs_load::load_gtfs(&args.gtfs_dir, &allowed_mots)
        .context("Failed to load GTFS for Pfaedle")?;

    // 2. Load OSM
    // Calculate BBox from GTFS stops
    // Collect all unique stop IDs from patterns
    let mut relevant_stop_ids = ahash::AHashSet::new();
    for pattern in gtfs_data.patterns.keys() {
        for stop_id in &pattern.stop_ids {
            relevant_stop_ids.insert(stop_id);
        }
    }

    let bbox = if relevant_stop_ids.is_empty() {
        None
    } else {
        let mut min_lat = f64::MAX;
        let mut max_lat = f64::MIN;
        let mut min_lon = f64::MAX;
        let mut max_lon = f64::MIN;

        for stop_id in relevant_stop_ids {
            if let Some(stop) = gtfs_data.gtfs.stops.get(stop_id) {
                min_lat = min_lat.min(stop.latitude.expect("Stop missing latitude"));
                max_lat = max_lat.max(stop.latitude.expect("Stop missing latitude"));
                min_lon = min_lon.min(stop.longitude.expect("Stop missing longitude"));
                max_lon = max_lon.max(stop.longitude.expect("Stop missing longitude"));
            }
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

    let osm_data = osm_load::load_osm(
        &args.osm_file,
        &gtfs_data.used_route_types,
        bbox,
        args.skip_small_roads,
    )?;

    // 3. Match
    let results = matcher::match_patterns(&gtfs_data, &osm_data);

    // 4. Write shapes.txt
    // If wipe_shapes is set, we need to carefully remove only shapes for the current MOTs
    // and keep the rest.
    // If wipe_shapes is FALSE, we might be appending duplicates? Or maybe we should overwrite
    // anyway? The original logic was: args.wipe_shapes -> delete file. Else -> append?
    // Actually, csv::Writer::from_path truncates by default unless we set append(true).
    // So logic was: if wipe_shapes -> delete file, then from_path (truncate/create) -> new file.
    // if !wipe_shapes -> from_path (truncate/create) -> overwrites unless we used OpenOptions.
    // Wait, `csv::Writer::from_path` ALWAYS truncates.
    // So previously, if `shapes.txt` existed and `wipe_shapes` was FALSE, it would effectively be wiped anyway?
    // Let's check std::fs::remove_file usage.
    // Ah, if `wipe_shapes` was false, lines 70-75 skipped.
    // Line 136: `csv::Writer::from_path(&shapes_path)?`
    // documentation says: "If the file already exists, it is overwritten."
    // So previously, it ALWAYS wiped shapes.txt, practically speaking.
    // The `wipe_shapes` arg was redundant or maybe intent was different?
    // Or maybe the user meant "if I don't say wipe, don't run pfaedle?" No.
    //
    // New logic:
    // If wipe_shapes:
    //   Read existing shapes.txt.
    //   Identify shape_ids to REMOVE. These are shape_ids used by trips of the selected MOTs.
    //   Filter existing records: Keep if shape_id NOT in remove_set.
    //   Write kept records + new results.
    // If !wipe_shapes:
    //   Presumably we just want to run for these MOTs. The user expectation "only wipe shapes for that specific mot"
    //   implies we KEEP others.
    //   So regardless of `wipe_shapes` flag (or maybe ONLY if it is set?), we want preservation behavior.
    //   Let's assume the user WANTS to update the shapes for the selected MOTs.
    //   So we ALWAYS need to "wipe" the old shapes for the selected MOTs (replace them) and keep others.
    //   "if MOTS is selected (and is not all), only wipe shapes for that specific mot route type."
    //   This implies: Read all, remove partial, add new.

    println!("Updating shapes.txt at {:?}", shapes_path);

    let mut existing_shapes = Vec::new();

    if shapes_path.exists() {
        println!("Reading existing shapes...");
        let mut rdr = csv::Reader::from_path(&shapes_path)?;
        for result in rdr.deserialize() {
            let record: ShapeRecord = result?;
            existing_shapes.push(record);
        }
    }

    // Determine which shape_ids belong to the *current* selection of MOTs (the ones we are processing).
    // We want to REPLACE these.
    // We also want to KEEP shape_ids that belong to OTHER MOTs.
    //
    // Issue: How do we know which shape_id belongs to which MOT in the OLD file?
    // We don't have that info in shapes.txt.
    // We can infer it from the *current* GTFS.
    //
    // Strategy:
    // 1. Identify all shape_ids used by trips in the GTFS that match `allowed_mots`.
    //    These are the "Target Shape IDs".
    // 2. Filter existing_shapes:
    //    If a shape's ID is in "Target Shape IDs", DROP IT (we are about to regenerate it).
    //    Else, KEEP IT.
    // 3. Append new results.

    let mut shapes_to_replace = ahash::AHashSet::new();
    if args.mots != "all" {
        // If specific MOTs selected, we only want to replace shapes for trips of those MOTs.
        for (trip_id, trip) in &gtfs_data.gtfs.trips {
            let route = gtfs_data.gtfs.routes.get(&trip.route_id);
            if let Some(r) = route {
                let cat = mots::map_route_type_to_category(r.route_type);
                if allowed_mots.contains(&cat) {
                    if let Some(sid) = &trip.shape_id {
                        shapes_to_replace.insert(sid.clone());
                    }
                }
            }
        }
        println!(
            "Identified {} shape_ids to replace based on selected MOTs.",
            shapes_to_replace.len()
        );

        // Also add the shape_ids that we just generated matching patterns for?
        // Actually, `results` contains the NEW shape_ids.
        // We want to remove OLD entries that *conflict* or are *obsolete* versions.
        // If we found a match, we generate a NEW shape_id usually (unless we reuse IDs?).
        // Pfaedle generates IDs like "shp_2_123...".
        // The old file might have "shp_2_..."
        // Safe bet: remove any shape_id that is referenced by the trips we are currently processing.
    } else {
        // If MOTs is "all", and we are wiping (or just running?), we probably want to replace EVERYTHING?
        // "if MOTS is selected (and is not all), only wipe shapes for that specific mot route type."
        // Implies if MOTS == all, we wipe everything (default behavior).
        if args.wipe_shapes {
            println!("MOTs='all' and wipe_shapes=true. Wiping all existing shapes.");
            existing_shapes.clear();
        }
    }

    // Filter existing
    if !shapes_to_replace.is_empty() {
        let before_count = existing_shapes.len();
        existing_shapes.retain(|s| !shapes_to_replace.contains(&s.shape_id));
        let after_count = existing_shapes.len();
        println!(
            "Pruned {} existing shape points (kept {}).",
            before_count - after_count,
            after_count
        );
    }

    // Write everything back
    let mut wtr = csv::Writer::from_path(&shapes_path)?;

    // 1. Write kept existing shapes
    for shape in existing_shapes {
        wtr.serialize(shape)?;
    }

    // 2. Write new shapes
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
        drop(wtr);
        std::fs::rename(routes_out_path, routes_path)?;
    }

    println!("Done!");

    Ok(())
}
