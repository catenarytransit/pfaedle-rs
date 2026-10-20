//! Stable shape identifiers within a run, with a new namespace for each run.
//!
//! IDs use the route short name, the ordered stop sequence and a 24-bit run
//! nonce. A separate nonce prevents an updated geometry from silently reusing
//! the ID of an earlier matching run.

use std::collections::hash_map::RandomState;
use std::hash::BuildHasher;
use std::path::Path;

use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result, bail};

use crate::gtfs_load::StopPattern;

const FNV_OFFSET: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x100000001b3;
const SHORT_MASK: u64 = 0x00ff_ffff;

/// The length prefix separates e.g. ["a", "bc"] from ["ab", "c"].
/// Hashing is deliberately independent of DefaultHasher and its implementation.
fn stop_hash(stops: &[String]) -> u64 {
    let mut hash = FNV_OFFSET;
    for stop in stops {
        for byte in (stop.len() as u64).to_le_bytes().into_iter().chain(stop.bytes()) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(FNV_PRIME);
        }
    }
    hash
}

/// Encode route names without conflating spaces, punctuation or UTF-8 bytes.
fn encode_name(name: &str) -> String {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut result = String::with_capacity(name.len());
    for byte in name.bytes() {
        if byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b'~') {
            result.push(char::from(byte));
        } else {
            result.push('%');
            result.push(char::from(HEX[(byte >> 4) as usize]));
            result.push(char::from(HEX[(byte & 15) as usize]));
        }
    }
    result
}

fn short_id(pattern: &StopPattern, run: u32) -> String {
    format!(
        "shp-{}-{:06x}-{run:06x}",
        encode_name(&pattern.route_name),
        stop_hash(&pattern.stop_ids) & SHORT_MASK,
    )
}

/// When the 24-bit stop digest collides inside a feed, use the full digest and
/// route ID. That also distinguishes separate routes with identical short names
/// and stop sequences, while keeping the compact format for ordinary patterns.
fn long_id(pattern: &StopPattern, run: u32) -> String {
    format!(
        "shp-{}-{:016x}-{}-{run:06x}",
        encode_name(&pattern.route_name),
        stop_hash(&pattern.stop_ids),
        encode_name(&pattern.route_id)
    )
}

/// Avoid reusing a previous run's namespace when updating an existing shapes.txt.
/// The scan stores only distinct 24-bit nonces, not the (potentially millions
/// of) geometry rows.
fn existing_run_ids(shapes_path: &Path) -> Result<AHashSet<u32>> {
    let mut used = AHashSet::new();
    if !shapes_path.exists() {
        return Ok(used);
    }

    let mut csv = csv::Reader::from_path(shapes_path)
        .with_context(|| format!("Cannot read {}", shapes_path.display()))?;
    let index = csv.headers()?
        .iter()
        .position(|header| header == "shape_id")
        .context("Existing shapes.txt has no shape_id column")?;

    for row in csv.records() {
        let record = row?;
        if let Some(shape_id) = record.get(index) {
            if !shape_id.starts_with("shp-") {
                continue;
            }
            if let Some((_, suffix)) = shape_id.rsplit_once('-') {
                if suffix.len() == 6 && suffix.bytes().all(|c| c.is_ascii_hexdigit()) {
                    if let Ok(nonce) = u32::from_str_radix(suffix, 16) {
                        used.insert(nonce);
                    }
                }
            }
        }
    }
    Ok(used)
}

fn choose_run_id(used: &AHashSet<u32>) -> Result<u32> {
    // RandomState obtains independently seeded random keys. Only 24 bits are
    // retained; existing namespaces are rejected before matching begins.
    for _ in 0..1024 {
        let nonce = (RandomState::new().hash_one("pfaedle-shape-run") as u32) & 0x00ff_ffff;
        if !used.contains(&nonce) {
            return Ok(nonce);
        }
    }
    bail!("Could not allocate an unused shape run ID")
}

/// Preassign IDs before either bus or non-bus workers write the binary cache.
/// Renaming IDs after matching would leave cached geometry under stale names.
pub fn assign_shape_ids<'a>(
    patterns: impl IntoIterator<Item = &'a StopPattern>,
    shapes_path: &Path,
) -> Result<AHashMap<StopPattern, String>> {
    let used = existing_run_ids(shapes_path)?;
    let run = choose_run_id(&used)?;

    let mut groups: AHashMap<String, Vec<&StopPattern>> = AHashMap::new();
    for pattern in patterns {
        groups.entry(short_id(pattern, run)).or_default().push(pattern);
    }

    let mut ids = AHashMap::new();
    let mut claimed = AHashMap::<String, &StopPattern>::new();

    for (compact, group) in groups {
        let ambiguous = group.len() > 1;
        for pattern in group {
            let id = if ambiguous {
                long_id(pattern, run)
            } else {
                compact.clone()
            };
            if let Some(other) = claimed.insert(id.clone(), pattern) {
                if other != pattern {
                    bail!(
                        "Cannot assign unique shape ID {id} for routes {} and {}",
                        other.route_id,
                        pattern.route_id
                    );
                }
            }
            ids.insert(pattern.clone(), id);
        }
    }
    Ok(ids)
}

#[cfg(test)]
mod tests {
    use super::*;
    use gtfs_structures::RouteType;

    fn p(route_id: &str, name: &str, stops: &[&str]) -> StopPattern {
        StopPattern {
            stop_ids: stops.iter().map(|stop| (*stop).to_owned()).collect(),
            route_type: Some(RouteType::Bus),
            route_id: route_id.to_owned(),
            route_name: name.to_owned(),
        }
    }

    #[test]
    fn compact_id_is_stable_and_versioned() {
        let pattern = p("blue", "Blue", &["STOP-1", "STOP-2", "STOP-3"]);
        assert_eq!(short_id(&pattern, 0x123abc), "shp-Blue-7b5692-123abc");
        assert_ne!(short_id(&pattern, 1), short_id(&pattern, 2));
        assert_eq!(short_id(&pattern, 1), short_id(&pattern, 1));
    }

    #[test]
    fn routes_and_directions_do_not_alias() {
        let a = p("blue", "Blue", &["A", "B", "C"]);
        assert_ne!(short_id(&a, 9), short_id(&p("red", "Red", &["A", "B", "C"]), 9));
        assert_ne!(short_id(&a, 9), short_id(&p("blue", "Blue", &["C", "B", "A"]), 9));
        assert_ne!(short_id(&a, 9), short_id(&p("blue", "Blue", &["A", "C"]), 9));
        assert_ne!(stop_hash(&vec!["a".into(), "bc".into()]),
                   stop_hash(&vec!["ab".into(), "c".into()]));
    }

    #[test]
    fn route_names_are_losslessly_encoded() {
        let a = p("a", "T&T 東京", &["1", "2"]);
        let b = p("b", "T-T 東京", &["1", "2"]);
        assert!(short_id(&a, 1).starts_with("shp-T%26T%20%E6%9D%B1%E4%BA%AC-"));
        assert_ne!(short_id(&a, 1), short_id(&b, 1));
    }

    #[test]
    fn route_id_disambiguates_identical_route_names_and_stops() {
        let a = p("agency-a", "10", &["A", "B"]);
        let b = p("agency-b", "10", &["A", "B"]);
        assert_eq!(short_id(&a, 1), short_id(&b, 1));
        assert_ne!(long_id(&a, 1), long_id(&b, 1));
    }

    #[test]
    fn chosen_nonce_excludes_previous_runs() {
        let mut used = AHashSet::new();
        used.insert(0x123abc);
        assert_ne!(choose_run_id(&used).unwrap(), 0x123abc);
    }
}
