use crate::mots::{self, MotCategory};
use ahash::{AHashMap, AHashSet};
use anyhow::Result;
use gtfs_structures::{Gtfs, RouteType};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, Clone)]
pub struct StopPattern {
    pub stop_ids: Vec<String>,
    pub route_type: Option<RouteType>,
}

// Implement Hash/Eq for StopPattern to use as key
impl PartialEq for StopPattern {
    fn eq(&self, other: &Self) -> bool {
        self.stop_ids == other.stop_ids && self.route_type == other.route_type
    }
}
impl Eq for StopPattern {}
impl std::hash::Hash for StopPattern {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.stop_ids.hash(state);
        self.route_type.hash(state);
    }
}

pub struct GtfsData {
    pub gtfs: Gtfs,
    // Map pattern to list of Trip references
    pub patterns: AHashMap<StopPattern, Vec<String>>, // Using TripId
    pub used_route_types: AHashSet<RouteType>,
}

fn faster_stop_time_reader_injection(mut gtfs: Gtfs, stop_times_path: &Path) -> Result<Gtfs> {
    let file = File::open(stop_times_path)?;
    let buf_reader = BufReader::new(file);
    let mut rdr = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_reader(buf_reader);

    let mut current_trip_id = String::new();
    let mut stop_times_buffer = Vec::with_capacity(1024);

    for result in rdr.deserialize() {
        let stop_time_raw: gtfs_structures::RawStopTime = result?;

        if !current_trip_id.is_empty() {
            if stop_time_raw.trip_id != current_trip_id {
                if let Some(trip) = gtfs.trips.get_mut(current_trip_id.as_str()) {
                    trip.stop_times.reserve_exact(stop_times_buffer.len());
                    for st in stop_times_buffer.drain(..) {
                        trip.stop_times.push(st);
                    }
                } else {
                    stop_times_buffer.clear();
                }
                current_trip_id = stop_time_raw.trip_id.clone();
            }
        } else {
            current_trip_id = stop_time_raw.trip_id.clone();
        }

        if let Some(stop) = gtfs.stops.get(stop_time_raw.stop_id.as_str()) {
            let stop_time = gtfs_structures::StopTime::from(stop_time_raw, stop.clone());
            stop_times_buffer.push(stop_time);
        }
    }

    if !current_trip_id.is_empty() {
        if let Some(trip) = gtfs.trips.get_mut(current_trip_id.as_str()) {
            trip.stop_times.reserve_exact(stop_times_buffer.len());
            for st in stop_times_buffer.drain(..) {
                trip.shape_id = trip.shape_id.clone();
                trip.stop_times.push(st);
            }
        }
    }

    Ok(gtfs)
}

pub fn load_gtfs(path: &Path, allowed_mots: &AHashSet<MotCategory>) -> Result<GtfsData> {
    println!("Loading GTFS from {:?}", path);
    let path_str = path.to_str().unwrap();
    println!("Loading GTFS structures for pfaedle from {}", path_str);

    let mut gtfs = gtfs_structures::GtfsReader::default()
        .read_shapes(false)
        .read_stop_times(false)
        .read(path_str)
        .map_err(|e| anyhow::anyhow!("Failed to read GTFS for pfaedle: {:?}", e))?;

    let shapes_missing = !path.join("shapes.txt").exists();
    if shapes_missing {
        println!("shapes.txt missing or invalid, ignoring and clearing shape_ids from trips.");
        for trip in gtfs.trips.values_mut() {
            trip.shape_id = None;
        }
    }

    // Pre-filter routes by allowed MOTs to avoid loading stop times for unused modes.
    gtfs.routes.retain(|_, route| {
        let cat = mots::map_route_type_to_category(route.route_type);
        allowed_mots.contains(&cat)
    });

    // Pre-filter trips to keep only those referencing the retained routes.
    gtfs.trips
        .retain(|_, trip| gtfs.routes.contains_key(&trip.route_id));

    println!("Injecting stop times...");
    let stop_times_path = path.join("stop_times.txt");
    let gtfs = faster_stop_time_reader_injection(gtfs, &stop_times_path)?;

    println!(
        "Loaded {} stops, {} trips",
        gtfs.stops.len(),
        gtfs.trips.len()
    );

    let mut patterns: AHashMap<StopPattern, Vec<String>> = AHashMap::new();

    for (trip_id, trip) in &gtfs.trips {
        let route = gtfs.routes.get(&trip.route_id);

        if route.is_none() {
            continue;
        }

        // Create vector of references to sort
        let mut stop_times: Vec<_> = trip.stop_times.iter().collect();
        stop_times.sort_unstable_by_key(|st| st.stop_sequence);

        let stop_ids: Vec<String> = stop_times.iter().map(|st| st.stop.id.clone()).collect();

        if stop_ids.is_empty() {
            continue;
        }

        let route_type = route.map(|r| r.route_type);

        let pattern = StopPattern {
            stop_ids,
            route_type,
        };
        patterns.entry(pattern).or_default().push(trip_id.clone());
    }

    let used_route_types: AHashSet<RouteType> =
        patterns.keys().filter_map(|p| p.route_type).collect();

    println!("Found {} unique patterns", patterns.len());
    println!("Used route types: {:?}", used_route_types);

    Ok(GtfsData {
        gtfs,
        patterns,
        used_route_types,
    })
}
