//! Match non-road modes in the same configuration groups as upstream pfaedle.
//!
//! C++ `PfaedleMain` constructs and destroys one OSM graph for each MOT config
//! instead of retaining rail, tram/subway, ferry and aerial networks together.
//! Keeping the same lifetime boundary both reduces peak RSS and makes the
//! profile-specific OSM filter/level/one-way rules unambiguous.

use ahash::{AHashMap, AHashSet};
use anyhow::Result;
use gtfs_structures::RouteType;

use crate::gtfs_load::{GtfsData, StopPattern};
use crate::matcher::{ShapeResult, match_patterns_full_graph};
use crate::mots::{MotCategory, map_route_type_to_category};
use crate::osm_load::load_osm;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum ProfileKey {
    Rail,
    TramSubway,
    Ferry,
    Gondola,
    Funicular,
    Other,
}

fn profile_key(pattern: &StopPattern) -> ProfileKey {
    let Some(route_type) = pattern.route_type else {
        return ProfileKey::Other;
    };
    match map_route_type_to_category(route_type) {
        MotCategory::Rail => ProfileKey::Rail,
        MotCategory::Tram | MotCategory::Subway => ProfileKey::TramSubway,
        MotCategory::Ferry => ProfileKey::Ferry,
        MotCategory::Gondola | MotCategory::CableCar => ProfileKey::Gondola,
        MotCategory::Funicular => ProfileKey::Funicular,
        _ => ProfileKey::Other,
    }
}

fn patterns_bbox(
    gtfs: &GtfsData,
    patterns: &[(&StopPattern, &Vec<String>)],
) -> Option<(f64, f64, f64, f64)> {
    let mut min_lat = f64::MAX;
    let mut max_lat = f64::MIN;
    let mut min_lon = f64::MAX;
    let mut max_lon = f64::MIN;
    let mut found = false;

    for (pattern, _) in patterns {
        for stop_id in &pattern.stop_ids {
            let Some(stop) = gtfs.gtfs.stops.get(stop_id) else {
                continue;
            };
            let (Some(lat), Some(lon)) = (stop.latitude, stop.longitude) else {
                continue;
            };
            min_lat = min_lat.min(lat);
            max_lat = max_lat.max(lat);
            min_lon = min_lon.min(lon);
            max_lon = max_lon.max(lon);
            found = true;
        }
    }

    if !found {
        return None;
    }

    // Preserve the existing Rust safety padding. Upstream derives its padding
    // from max speed/hop distance; changing that policy is outside this patch.
    const PADDING_DEGREES: f64 = 0.5;
    Some((
        min_lon - PADDING_DEGREES,
        min_lat - PADDING_DEGREES,
        max_lon + PADDING_DEGREES,
        max_lat + PADDING_DEGREES,
    ))
}

pub fn match_nonbus_profiles(
    gtfs: &GtfsData,
    osm_path: &std::path::Path,
    patterns: Vec<(&StopPattern, &Vec<String>)>,
    cache_file_path: &std::path::Path,
    match_threads: Option<usize>,
) -> Result<AHashMap<StopPattern, ShapeResult>> {
    let mut grouped: AHashMap<ProfileKey, Vec<(&StopPattern, &Vec<String>)>> = AHashMap::new();
    for (pattern, trips) in patterns {
        grouped
            .entry(profile_key(pattern))
            .or_default()
            .push((pattern, trips));
    }

    let mut keys: Vec<_> = grouped.keys().copied().collect();
    keys.sort();
    let mut all_results = AHashMap::new();

    for key in keys {
        let Some(profile_patterns) = grouped.remove(&key) else {
            continue;
        };
        if profile_patterns.is_empty() {
            continue;
        }

        let mut route_types: AHashSet<RouteType> = AHashSet::new();
        for (pattern, _) in &profile_patterns {
            if let Some(route_type) = pattern.route_type {
                route_types.insert(route_type);
            }
        }
        let bbox = patterns_bbox(gtfs, &profile_patterns);
        println!(
            "Loading {:?} OSM graph for {} patterns, bbox {:?}",
            key,
            profile_patterns.len(),
            bbox
        );

        // `load_osm` owns the graph; it is dropped at the end of this loop just
        // as the local C++ `trgraph::Graph graph` is destroyed after each motCfg.
        let osm = load_osm(osm_path, &route_types, bbox, false)?;
        let results = match_patterns_full_graph(
            gtfs,
            &osm,
            profile_patterns,
            cache_file_path,
            match_threads,
        )?;
        all_results.extend(results);
    }

    Ok(all_results)
}
