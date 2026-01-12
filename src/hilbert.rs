//! Hilbert curve utilities for spatial locality optimization.
//!
//! By sorting patterns by their Hilbert index, we ensure that
//! geographically proximate routes are processed together,
//! maximizing tile cache locality.

/// Convert lat/lon to Hilbert curve index.
/// Uses a 16-bit Hilbert curve (65536 x 65536 grid).
pub fn hilbert_index(lon: f64, lat: f64) -> u64 {
    // Normalize to 0..65535 range
    let x = ((lon + 180.0) / 360.0 * 65536.0).clamp(0.0, 65535.0) as u32;
    let y = ((lat + 90.0) / 180.0 * 65536.0).clamp(0.0, 65535.0) as u32;

    xy_to_hilbert(16, x, y)
}

/// Convert x,y coordinates to Hilbert index.
/// Based on the standard Hilbert curve algorithm.
fn xy_to_hilbert(order: u32, x: u32, y: u32) -> u64 {
    let n = 1u32 << order;
    let mut rx: u32;
    let mut ry: u32;
    let mut d: u64 = 0;
    let mut x = x;
    let mut y = y;

    let mut s = n / 2;
    while s > 0 {
        rx = if (x & s) > 0 { 1 } else { 0 };
        ry = if (y & s) > 0 { 1 } else { 0 };
        d += (s as u64 * s as u64) * ((3 * rx) ^ ry) as u64;

        // Rotate quadrant
        if ry == 0 {
            if rx == 1 {
                x = (n - 1) - x;
                y = (n - 1) - y;
            }
            std::mem::swap(&mut x, &mut y);
        }

        s /= 2;
    }

    d
}

/// Compute centroid of a set of points.
pub fn compute_centroid(points: &[(f64, f64)]) -> (f64, f64) {
    if points.is_empty() {
        return (0.0, 0.0);
    }

    let sum_lon: f64 = points.iter().map(|(lon, _)| lon).sum();
    let sum_lat: f64 = points.iter().map(|(_, lat)| lat).sum();
    let n = points.len() as f64;

    (sum_lon / n, sum_lat / n)
}

/// Compute Hilbert index for a route based on its stop centroids.
pub fn route_hilbert_index(stop_coords: &[(f64, f64)]) -> u64 {
    let (lon, lat) = compute_centroid(stop_coords);
    hilbert_index(lon, lat)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hilbert_basic() {
        // Points close together should have similar indices
        let idx1 = hilbert_index(-122.4, 37.8);
        let idx2 = hilbert_index(-122.35, 37.85);
        let idx3 = hilbert_index(0.0, 51.5); // London - far away

        // SF points should be closer to each other than to London
        let diff_sf = (idx1 as i64 - idx2 as i64).abs();
        let diff_london = (idx1 as i64 - idx3 as i64).abs();

        assert!(diff_sf < diff_london);
    }

    #[test]
    fn test_hilbert_ordering() {
        // Create a grid of points
        let mut points: Vec<(f64, f64, u64)> = Vec::new();
        for lon in (-180..180).step_by(30) {
            for lat in (-90..90).step_by(30) {
                let idx = hilbert_index(lon as f64, lat as f64);
                points.push((lon as f64, lat as f64, idx));
            }
        }

        // Sort by Hilbert index
        points.sort_by_key(|(_, _, idx)| *idx);

        // Check that consecutive points are relatively close
        for window in points.windows(2) {
            let (lon1, lat1, _) = window[0];
            let (lon2, lat2, _) = window[1];
            let dist = ((lon2 - lon1).powi(2) + (lat2 - lat1).powi(2)).sqrt();
            // Most consecutive points should be within ~60 degrees
            // (allowing for curve crossings)
            assert!(dist < 100.0, "Jump too large: {} degrees", dist);
        }
    }

    #[test]
    fn test_centroid() {
        let points = vec![(-122.0, 37.0), (-122.0, 38.0), (-123.0, 37.5)];
        let (lon, lat) = compute_centroid(&points);
        assert!((lon - (-122.333)).abs() < 0.01);
        assert!((lat - 37.5).abs() < 0.01);
    }
}
