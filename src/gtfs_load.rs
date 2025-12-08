use crate::mots::{self, MotCategory};
use ahash::{AHashMap, AHashSet};
use anyhow::Result;
use gtfs_structures::{Gtfs, RouteType};
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

pub fn load_gtfs(path: &Path, allowed_mots: &AHashSet<MotCategory>) -> Result<GtfsData> {
    println!("Loading GTFS from {:?}", path);
    // Use ? to extract the Gtfs result, then wrap it in the GtfsData struct.
    // NOTE: gtfs_structures::Gtfs::new returns Result<Gtfs, Error>
    // Load raw GTFS data first
    let path_str = path.to_str().unwrap();
    let mut raw = gtfs_structures::RawGtfs::from_path(path_str)
        .map_err(|e| anyhow::anyhow!("Failed to load raw GTFS: {}", e))?;

    // If shapes.txt is missing/invalid, ignore it and remove shape references from trips
    let shapes_missing = match &raw.shapes {
        Some(Ok(_)) => false,
        _ => true,
    };

    if shapes_missing {
        println!("shapes.txt missing or invalid, ignoring and clearing shape_ids from trips.");
        raw.shapes = Some(Ok(vec![]));
        if let Ok(ref mut trips) = raw.trips {
            for trip in trips {
                trip.shape_id = None;
            }
        }
    }

    let gtfs = Gtfs::try_from(raw).map_err(|e| anyhow::anyhow!("Failed to process GTFS: {}", e))?;

    println!(
        "Loaded {} stops, {} trips",
        gtfs.stops.len(),
        gtfs.trips.len()
    );

    let mut patterns: AHashMap<StopPattern, Vec<String>> = AHashMap::new();

    for (trip_id, trip) in &gtfs.trips {
        let route = gtfs.routes.get(&trip.route_id);

        // Filter by MOT
        if let Some(r) = route {
            let cat = mots::map_route_type_to_category(r.route_type);
            if !allowed_mots.contains(&cat) {
                continue;
            }
        } else {
            // If no route found, maybe skip? or process? safe to skip probably.
            continue;
        }

        // Sort stop_times by sequence just in case
        let mut stop_times = trip.stop_times.clone();
        stop_times.sort_by_key(|st| st.stop_sequence);

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
