use anyhow::Result;
use gtfs_structures::{Gtfs, RouteType};
use std::collections::HashMap;
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
    pub patterns: HashMap<StopPattern, Vec<String>>, // Using TripId
}

pub fn load_gtfs(path: &Path) -> Result<GtfsData> {
    println!("Loading GTFS from {:?}", path);
    // Use ? to extract the Gtfs result, then wrap it in the GtfsData struct.
    // NOTE: gtfs_structures::Gtfs::new returns Result<Gtfs, Error>
    let gtfs = Gtfs::new(path.to_str().unwrap())
        .map_err(|e| anyhow::anyhow!("Failed to load GTFS: {}", e))?;

    println!(
        "Loaded {} stops, {} trips",
        gtfs.stops.len(),
        gtfs.trips.len()
    );

    let mut patterns: HashMap<StopPattern, Vec<String>> = HashMap::new();

    for (trip_id, trip) in &gtfs.trips {
        // Sort stop_times by sequence just in case
        let mut stop_times = trip.stop_times.clone();
        stop_times.sort_by_key(|st| st.stop_sequence);

        let stop_ids: Vec<String> = stop_times.iter().map(|st| st.stop.id.clone()).collect();

        if stop_ids.is_empty() {
            continue;
        }

        let route = gtfs.routes.get(&trip.route_id);
        let route_type = route.map(|r| r.route_type);

        let pattern = StopPattern {
            stop_ids,
            route_type,
        };
        patterns.entry(pattern).or_default().push(trip_id.clone());
    }

    println!("Found {} unique patterns", patterns.len());

    Ok(GtfsData { gtfs, patterns })
}
