//! Segment-by-segment shape matching for bus/coach routes.
//!
//! Each consecutive stop pair is matched independently using only
//! the tiles needed for that segment, then all segments are joined.

use ahash::{AHashMap, AHashSet};
use anyhow::Result;
use geo::Point;
use geo::algorithm::HaversineDistance;

use crate::graph::{EdgePL, Graph, MODE_BUS, NodeIndex, NodePL};
use crate::osm_load::SpatialNode;
use crate::pathfinding::{self, PathfinderContext, TransitMatch};
use crate::tile::{MergedTileData, TILE_SIZE, TileCache, TileCoord, compute_corridor_tiles};

/// Maximum number of tiles to load at once for a single segment.
const MAX_TILES_PER_SEGMENT: usize = 20;

/// If a corridor exceeds this many tiles, use streaming approach.
const STREAM_THRESHOLD: usize = 8;

/// Result of matching a segment between two stops.
pub struct SegmentResult {
    pub geometry: Vec<(f64, f64)>, // lat, lon pairs
    pub cost: f64,
}

/// Segment-by-segment matcher using tiled graph loading.
pub struct SegmentMatcher<'a> {
    tile_cache: &'a mut TileCache,
    ctx: PathfinderContext,
}

impl<'a> SegmentMatcher<'a> {
    pub fn new(tile_cache: &'a mut TileCache) -> Self {
        Self {
            tile_cache,
            ctx: PathfinderContext::new(),
        }
    }

    /// Match a complete route by joining segments.
    pub fn match_route(
        &mut self,
        stop_coords: &[Point<f64>],
        preferred_match: Option<&TransitMatch>,
    ) -> Option<Vec<(f64, f64)>> {
        if stop_coords.len() < 2 {
            return None;
        }

        let mut full_geometry = Vec::new();

        // Add first stop
        full_geometry.push((stop_coords[0].y(), stop_coords[0].x()));

        for window in stop_coords.windows(2) {
            let p1 = window[0];
            let p2 = window[1];

            // Match this segment
            if let Some(segment) = self.match_segment(p1, p2, preferred_match) {
                // Skip first point of segment (it's the last point of previous)
                full_geometry.extend(segment.geometry.into_iter().skip(1));
            } else {
                // Fallback: straight line
                full_geometry.push((p2.y(), p2.x()));
            }
        }

        Some(full_geometry)
    }

    /// Match a single segment between two stops.
    fn match_segment(
        &mut self,
        p1: Point<f64>,
        p2: Point<f64>,
        preferred_match: Option<&TransitMatch>,
    ) -> Option<SegmentResult> {
        // Determine tiles needed
        let t1 = TileCoord::from_point(p1.x(), p1.y());
        let t2 = TileCoord::from_point(p2.x(), p2.y());

        let corridor = compute_corridor_tiles(p1, p2);

        if corridor.len() > STREAM_THRESHOLD {
            // Use streaming approach for distant stops
            self.match_segment_streaming(p1, p2, &corridor, preferred_match)
        } else {
            // Load all tiles at once
            self.match_segment_direct(p1, p2, preferred_match)
        }
    }

    /// Direct matching when tiles fit in memory.
    fn match_segment_direct(
        &mut self,
        p1: Point<f64>,
        p2: Point<f64>,
        preferred_match: Option<&TransitMatch>,
    ) -> Option<SegmentResult> {
        let merged = self.tile_cache.get_for_segment(p1, p2).ok()?;

        self.pathfind_in_tiles(&merged, p1, p2, preferred_match)
    }

    /// Streaming approach for distant stops.
    fn match_segment_streaming(
        &mut self,
        p1: Point<f64>,
        p2: Point<f64>,
        corridor: &[TileCoord],
        preferred_match: Option<&TransitMatch>,
    ) -> Option<SegmentResult> {
        let mut full_geometry = Vec::new();
        let mut current_pos = p1;
        let mut total_cost = 0.0;

        // Process corridor in windows of 3 tiles
        let window_size = 3;
        let mut i = 0;

        while i < corridor.len() {
            let end_idx = (i + window_size).min(corridor.len());
            let window_tiles: Vec<TileCoord> = corridor[i..end_idx].to_vec();

            // Load this window
            let merged = self.tile_cache.merge_tiles_direct(&window_tiles).ok()?;

            // Determine target: final point if last window, else edge of window
            let is_last_window = end_idx >= corridor.len();
            let target = if is_last_window {
                p2
            } else {
                // Find a point near the boundary of the current window
                self.find_window_exit(&merged, current_pos, p2)?
            };

            // Pathfind within window
            if let Some(segment) =
                self.pathfind_in_tiles(&merged, current_pos, target, preferred_match)
            {
                if full_geometry.is_empty() {
                    full_geometry.extend(segment.geometry);
                } else {
                    full_geometry.extend(segment.geometry.into_iter().skip(1));
                }
                total_cost += segment.cost;
                current_pos = Point::new(
                    full_geometry.last().unwrap().1,
                    full_geometry.last().unwrap().0,
                );
            } else {
                // Failed to pathfind in this window
                return None;
            }

            if is_last_window {
                break;
            }

            // Move to next window with overlap
            i += window_size - 1;
        }

        Some(SegmentResult {
            geometry: full_geometry,
            cost: total_cost,
        })
    }

    /// Find a good exit point from the current tile window.
    fn find_window_exit(
        &self,
        merged: &MergedTileData,
        current: Point<f64>,
        final_target: Point<f64>,
    ) -> Option<Point<f64>> {
        // Find node closest to the line toward final target,
        // but near the edge of the current merged area
        let direction = Point::new(
            final_target.x() - current.x(),
            final_target.y() - current.y(),
        );

        // Sample a point ~75% of the way through current window
        let progress = 0.75;
        let sample = Point::new(
            current.x() + direction.x() * progress,
            current.y() + direction.y() * progress,
        );

        // Find nearest node to sample point
        let nearest = merged
            .spatial_tree
            .nearest_neighbor(&[sample.x(), sample.y()])?;

        let node = &merged.graph.node(nearest.index).payload;
        Some(node.point)
    }

    /// Pathfind between two points within merged tile data.
    fn pathfind_in_tiles(
        &mut self,
        merged: &MergedTileData,
        p1: Point<f64>,
        p2: Point<f64>,
        preferred_match: Option<&TransitMatch>,
    ) -> Option<SegmentResult> {
        // Find nearest nodes to start and end
        let start_candidates: Vec<_> = merged
            .spatial_tree
            .nearest_neighbor_iter(&[p1.x(), p1.y()])
            .take(10)
            .collect();

        let end_candidates: Vec<_> = merged
            .spatial_tree
            .nearest_neighbor_iter(&[p2.x(), p2.y()])
            .take(10)
            .collect();

        if start_candidates.is_empty() || end_candidates.is_empty() {
            return None;
        }

        // Try combinations to find best path
        let mut best_path: Option<(f64, Vec<usize>)> = None;
        let mut best_start = start_candidates[0].index;
        let mut best_end = end_candidates[0].index;

        for start in &start_candidates[..start_candidates.len().min(5)] {
            for end in &end_candidates[..end_candidates.len().min(5)] {
                if start.index == end.index {
                    // Same node
                    if best_path.is_none() {
                        best_path = Some((0.0, vec![]));
                        best_start = start.index;
                        best_end = end.index;
                    }
                    continue;
                }

                self.ctx.reset();
                if let Some((cost, edges)) = pathfinding::pathfind_with_context(
                    &mut self.ctx,
                    &merged.graph,
                    start.index,
                    end.index,
                    MODE_BUS,
                    None,
                    preferred_match,
                ) {
                    if best_path.is_none() || cost < best_path.as_ref().unwrap().0 {
                        best_path = Some((cost, edges));
                        best_start = start.index;
                        best_end = end.index;
                    }
                }
            }
        }

        // Build geometry from best path
        let (cost, edges) = best_path?;
        let mut geometry = Vec::new();

        // Add start point
        let start_node = &merged.graph.node(best_start).payload;
        geometry.push((start_node.point.y(), start_node.point.x()));

        // Add edge geometries
        for edge_idx in edges {
            let edge = merged.graph.edge(edge_idx);
            for coord in edge.payload.geometry.coords().skip(1) {
                geometry.push((coord.y, coord.x));
            }
        }

        Some(SegmentResult { geometry, cost })
    }
}

impl TileCache {
    /// Direct tile merging for segment matcher.
    pub fn merge_tiles_direct(&mut self, coords: &[TileCoord]) -> Result<MergedTileData> {
        self.merge_tiles(coords)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segment_result_structure() {
        let segment = SegmentResult {
            geometry: vec![(37.8, -122.4), (37.85, -122.35)],
            cost: 100.0,
        };
        assert_eq!(segment.geometry.len(), 2);
    }
}
