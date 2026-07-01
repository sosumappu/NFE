//! Offline raceline preview support for simulator track JSON files.
//!
//! This module adapts track files into pure `TrackMap` DTOs. It deliberately
//! stops before solving or rendering so binaries can remain thin wiring over
//! `nfe-algo` and the runtime Foxglove/MCAP adapters.

use std::path::Path;

use anyhow::Result;
use nfe_core::mapping::{BoundarySet, OccupancyGrid, TrackMap};
use nfe_core::{Point2, WallLine};
use serde::Deserialize;

const DEFAULT_RASTER_RESOLUTION_M: f32 = 0.05;
const DEFAULT_RASTER_MARGIN_M: f32 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackFrame {
    World,
    Start,
}

impl TrackFrame {
    pub fn as_str(self) -> &'static str {
        match self {
            TrackFrame::World => "world",
            TrackFrame::Start => "start",
        }
    }
}

#[derive(Deserialize)]
struct JsonTrack {
    #[serde(alias = "inner_wall")]
    inner_walls: Vec<[f32; 2]>,
    #[serde(alias = "outer_wall")]
    outer_walls: Vec<[f32; 2]>,
    start: JsonStart,
}

#[derive(Clone, Copy, Deserialize)]
struct JsonStart {
    x: f32,
    y: f32,
    yaw_rad: f32,
}

pub fn load_track_map(path: impl AsRef<Path>, frame: TrackFrame) -> Result<TrackMap> {
    let raw = std::fs::read_to_string(path)?;
    let track: JsonTrack = serde_json::from_str(&raw)?;
    Ok(track_map(&track, frame))
}

fn track_map(track: &JsonTrack, frame: TrackFrame) -> TrackMap {
    let inner = transformed_loop(&track.inner_walls, track.start, frame);
    let outer = transformed_loop(&track.outer_walls, track.start, frame);
    let mut walls = Vec::with_capacity(inner.len() + outer.len());
    walls.extend(loop_to_walls(&inner));
    walls.extend(loop_to_walls(&outer));
    let occupancy = rasterize_track(
        &inner,
        &outer,
        DEFAULT_RASTER_RESOLUTION_M,
        DEFAULT_RASTER_MARGIN_M,
    );
    TrackMap {
        boundaries: BoundarySet { walls },
        occupancy,
        complete: true,
        revision: 1,
        ..Default::default()
    }
}

fn transformed_loop(points: &[[f32; 2]], start: JsonStart, frame: TrackFrame) -> Vec<Point2> {
    points
        .iter()
        .map(|point| point_in_frame(*point, start, frame))
        .collect()
}

fn loop_to_walls(points: &[Point2]) -> Vec<WallLine> {
    if points.len() < 2 {
        return Vec::new();
    }
    (0..points.len())
        .map(|i| wall(points[i], points[(i + 1) % points.len()]))
        .collect()
}

fn rasterize_track(
    inner: &[Point2],
    outer: &[Point2],
    resolution_m: f32,
    margin_m: f32,
) -> Option<OccupancyGrid> {
    if inner.len() < 3 || outer.len() < 3 || !resolution_m.is_finite() || resolution_m <= 0.0 {
        return None;
    }
    let mut points = inner.iter().chain(outer.iter()).copied();
    let first = points.next()?;
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (first.x, first.x, first.y, first.y);
    for point in points {
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_y = min_y.min(point.y);
        max_y = max_y.max(point.y);
    }
    min_x -= margin_m;
    max_x += margin_m;
    min_y -= margin_m;
    max_y += margin_m;

    let width = ((max_x - min_x) / resolution_m).ceil().max(1.0) as u32;
    let height = ((max_y - min_y) / resolution_m).ceil().max(1.0) as u32;
    let mut cells = vec![1.0; width as usize * height as usize];
    for y in 0..height as usize {
        for x in 0..width as usize {
            let point = Point2::new(
                min_x + (x as f32 + 0.5) * resolution_m,
                min_y + (y as f32 + 0.5) * resolution_m,
            );
            if point_in_polygon(point, inner) ^ point_in_polygon(point, outer) {
                cells[y * width as usize + x] = -1.0;
            }
        }
    }

    Some(OccupancyGrid {
        origin: Point2::new(min_x, min_y),
        resolution_m,
        width,
        height,
        cells,
    })
}

fn point_in_frame(point: [f32; 2], start: JsonStart, frame: TrackFrame) -> Point2 {
    match frame {
        TrackFrame::World => Point2::new(point[0], point[1]),
        TrackFrame::Start => {
            let dx = point[0] - start.x;
            let dy = point[1] - start.y;
            let (s, c) = start.yaw_rad.sin_cos();
            Point2::new(c * dx + s * dy, -s * dx + c * dy)
        }
    }
}

fn point_in_polygon(point: Point2, polygon: &[Point2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = polygon.len() - 1;
    for i in 0..polygon.len() {
        let pi = polygon[i];
        let pj = polygon[j];
        if (pi.y > point.y) != (pj.y > point.y) {
            let x_intersection = (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x;
            if point.x < x_intersection {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn wall(p0: Point2, p1: Point2) -> WallLine {
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let len = dx.hypot(dy).max(1e-6);
    let nx = -dy / len;
    let ny = dx / len;
    WallLine {
        nx,
        ny,
        d: nx * p0.x + ny * p0.y,
        p0,
        p1,
        support: 1.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn start_frame_places_start_pose_at_origin() {
        let start = JsonStart {
            x: 1.0,
            y: 2.0,
            yaw_rad: 0.0,
        };

        let p = point_in_frame([2.0, 3.0], start, TrackFrame::Start);

        assert_eq!(p, Point2::new(1.0, 1.0));
    }

    #[test]
    fn track_map_closes_wall_loops() {
        let track = JsonTrack {
            inner_walls: vec![[0.0, 1.0], [1.0, 1.0]],
            outer_walls: vec![[0.0, -1.0], [1.0, -1.0]],
            start: JsonStart {
                x: 0.0,
                y: 0.0,
                yaw_rad: 0.0,
            },
        };

        let map = track_map(&track, TrackFrame::World);

        assert_eq!(map.boundaries.walls.len(), 4);
        assert!(map.occupancy.is_none());
        assert!(map.complete);
    }

    #[test]
    fn minispa_rasterization_has_expected_area_and_one_free_component() {
        let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let track_path = manifest_dir.join("../../worlds/tracks/minispa.json");
        let raw = std::fs::read_to_string(track_path).unwrap();
        let track: JsonTrack = serde_json::from_str(&raw).unwrap();
        let map = track_map(&track, TrackFrame::World);
        let occupancy = map.occupancy.as_ref().unwrap();
        let free_cells = occupancy.cells.iter().filter(|cell| **cell < 0.0).count();
        let free_area_m2 = free_cells as f32 * occupancy.resolution_m * occupancy.resolution_m;
        let inner = transformed_loop(&track.inner_walls, track.start, TrackFrame::World);
        let outer = transformed_loop(&track.outer_walls, track.start, TrackFrame::World);
        let expected_area_m2 = (signed_area(&inner).abs() - signed_area(&outer).abs()).abs();
        let relative_error = (free_area_m2 - expected_area_m2).abs() / expected_area_m2.max(1.0);

        assert!(free_cells > 1_000);
        assert!(
            relative_error < 0.08,
            "free_area={free_area_m2} expected={expected_area_m2}"
        );
        assert_eq!(free_component_count(occupancy), 1);
    }

    fn signed_area(points: &[Point2]) -> f32 {
        if points.len() < 3 {
            return 0.0;
        }
        let mut twice_area = 0.0;
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            twice_area += a.x * b.y - b.x * a.y;
        }
        0.5 * twice_area
    }

    fn free_component_count(grid: &OccupancyGrid) -> usize {
        let width = grid.width as usize;
        let height = grid.height as usize;
        let mut seen = vec![false; grid.cells.len()];
        let mut components = 0;
        for idx in 0..grid.cells.len() {
            if seen[idx] || grid.cells[idx] >= 0.0 {
                continue;
            }
            components += 1;
            let mut queue = VecDeque::new();
            seen[idx] = true;
            queue.push_back(idx);
            while let Some(cell) = queue.pop_front() {
                let x = cell % width;
                let y = cell / width;
                for (dx, dy) in [(1_i32, 0_i32), (-1, 0), (0, 1), (0, -1)] {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                        continue;
                    }
                    let nidx = ny as usize * width + nx as usize;
                    if !seen[nidx] && grid.cells[nidx] < 0.0 {
                        seen[nidx] = true;
                        queue.push_back(nidx);
                    }
                }
            }
        }
        components
    }
}
