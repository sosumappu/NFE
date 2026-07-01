//! Pure Foxglove scene builders used by runtime sinks and offline preview tools.
//!
//! This module has no files, sockets, channels, or pipeline state. It only turns
//! domain DTOs into Foxglove protobuf DTOs so the same visual representation is
//! shared by live MCAP recording and standalone tooling.

use nfe_core::mapping::TrackMap;
use nfe_core::raceline::RaceLine;
use nfe_core::telemetry::{WallKind, WorldSnapshotTelemetry};

pub mod pb_fg {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

pub const FOXGLOVE_DESCRIPTOR: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/foxglove_descriptor.bin"));

pub fn build_map_scene(timestamp_us: u64, frame_id: &str, map: &TrackMap) -> pb_fg::SceneUpdate {
    let mut entity = pb_fg::SceneEntity {
        id: "mapping_global_map".to_string(),
        timestamp: Some(timestamp(timestamp_us)),
        frame_id: frame_id.to_string(),
        frame_locked: 1,
        lines: Vec::with_capacity(map.boundaries.walls.len()),
        arrows: Vec::new(),
        spheres: Vec::new(),
    };

    for wall in &map.boundaries.walls {
        let mid_y = 0.5 * (wall.p0.y + wall.p1.y);
        let wall_color = if mid_y >= 0.0 {
            color(0.1, 0.5, 1.0, 0.9)
        } else {
            color(1.0, 0.55, 0.1, 0.9)
        };
        entity.lines.push(line_primitive(
            vec![
                vector(wall.p0.x, wall.p0.y, 0.02),
                vector(wall.p1.x, wall.p1.y, 0.02),
            ],
            0.01 + 0.02 * f64::from(wall.support.clamp(0.0, 1.0)),
            wall_color,
        ));
    }

    pb_fg::SceneUpdate {
        entities: vec![entity],
        deletions: Vec::new(),
    }
}

pub fn build_raceline_scene(
    timestamp_us: u64,
    frame_id: &str,
    line: &RaceLine,
) -> pb_fg::SceneUpdate {
    let points = line
        .points
        .iter()
        .map(|p| vector(p.p.x, p.p.y, 0.05))
        .collect();
    let lines = if line.points.is_empty() {
        Vec::new()
    } else {
        vec![line_primitive(points, 0.035, color(0.1, 1.0, 0.2, 0.95))]
    };
    let spheres = line
        .points
        .first()
        .map(|p| {
            vec![sphere(
                p.p.x,
                p.p.y,
                0.08,
                0.18,
                color(1.0, 0.85, 0.1, 0.95),
            )]
        })
        .unwrap_or_default();

    pb_fg::SceneUpdate {
        entities: vec![pb_fg::SceneEntity {
            id: "planning_raceline".to_string(),
            timestamp: Some(timestamp(timestamp_us)),
            frame_id: frame_id.to_string(),
            frame_locked: 1,
            lines,
            arrows: Vec::new(),
            spheres,
        }],
        deletions: Vec::new(),
    }
}

pub fn build_world_walls_scene(world: &WorldSnapshotTelemetry) -> pb_fg::SceneUpdate {
    let mut entity = pb_fg::SceneEntity {
        id: "world_walls".to_string(),
        timestamp: Some(timestamp(world.timestamp_us)),
        frame_id: world.frame_id.clone(),
        frame_locked: 1,
        lines: Vec::new(),
        arrows: Vec::new(),
        spheres: Vec::new(),
    };
    for wall in &world.walls {
        let color = match wall.kind {
            WallKind::Inner => color(0.1, 0.8, 1.0, 1.0),
            WallKind::Outer => color(1.0, 0.5, 0.1, 1.0),
            WallKind::Unknown => color(0.8, 0.8, 0.8, 1.0),
        };
        entity.lines.push(line_primitive(
            vec![
                vector(wall.p0.x, wall.p0.y, 0.0),
                vector(wall.p1.x, wall.p1.y, 0.0),
            ],
            0.02,
            color,
        ));
    }
    pb_fg::SceneUpdate {
        entities: vec![entity],
        deletions: Vec::new(),
    }
}

pub fn timestamp(timestamp_us: u64) -> pb_fg::Timestamp {
    pb_fg::Timestamp {
        sec: (timestamp_us / 1_000_000) as i32,
        nsec: ((timestamp_us % 1_000_000) * 1_000) as u32,
    }
}

fn line_primitive(
    points: Vec<pb_fg::Vector3>,
    thickness: f64,
    color: pb_fg::Color,
) -> pb_fg::LinePrimitive {
    pb_fg::LinePrimitive {
        r#type: 0,
        pose: Some(pose(nfe_core::Pose2::default())),
        thickness,
        scale_invariant_thickness: 0.0,
        color: Some(color),
        points,
        colors: Vec::new(),
        indices: Vec::new(),
    }
}

fn sphere(x: f32, y: f32, z: f32, diameter: f32, color: pb_fg::Color) -> pb_fg::SpherePrimitive {
    pb_fg::SpherePrimitive {
        pose: Some(pose_xyz_yaw(x, y, z, 0.0)),
        size: Some(vector(diameter, diameter, diameter)),
        color: Some(color),
    }
}

fn pose(p: nfe_core::Pose2) -> pb_fg::Pose {
    pose_xyz_yaw(p.x, p.y, 0.0, p.yaw)
}

fn pose_xyz_yaw(x: f32, y: f32, z: f32, yaw: f32) -> pb_fg::Pose {
    pb_fg::Pose {
        position: Some(vector(x, y, z)),
        orientation: Some(yaw_quaternion(yaw)),
    }
}

fn yaw_quaternion(yaw: f32) -> pb_fg::Quaternion {
    let half = 0.5 * yaw as f64;
    pb_fg::Quaternion {
        x: 0.0,
        y: 0.0,
        z: half.sin(),
        w: half.cos(),
    }
}

fn vector(x: f32, y: f32, z: f32) -> pb_fg::Vector3 {
    pb_fg::Vector3 {
        x: x as f64,
        y: y as f64,
        z: z as f64,
    }
}

fn color(r: f32, g: f32, b: f32, a: f32) -> pb_fg::Color {
    pb_fg::Color { r, g, b, a }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::mapping::{BoundarySet, TrackMap};
    use nfe_core::raceline::{RaceLine, RaceLinePoint};
    use nfe_core::{Point2, WallLine};

    #[test]
    fn map_scene_contains_one_line_per_wall() {
        let map = TrackMap {
            boundaries: BoundarySet {
                walls: vec![WallLine {
                    nx: 0.0,
                    ny: 1.0,
                    d: 0.5,
                    p0: Point2::new(0.0, 0.5),
                    p1: Point2::new(1.0, 0.5),
                    support: 1.0,
                }],
            },
            complete: true,
            revision: 1,
            ..Default::default()
        };

        let scene = build_map_scene(1_000, "map", &map);

        assert_eq!(scene.entities.len(), 1);
        assert_eq!(scene.entities[0].frame_id, "map");
        assert_eq!(scene.entities[0].lines.len(), 1);
        assert_eq!(scene.entities[0].lines[0].points.len(), 2);
    }

    #[test]
    fn raceline_scene_contains_polyline() {
        let line = RaceLine {
            points: vec![
                RaceLinePoint {
                    p: Point2::new(0.0, 0.0),
                    ..Default::default()
                },
                RaceLinePoint {
                    p: Point2::new(1.0, 0.0),
                    s_m: 1.0,
                    ..Default::default()
                },
            ],
            closed: true,
            revision: 1,
        };

        let scene = build_raceline_scene(1_000, "map", &line);

        assert_eq!(scene.entities.len(), 1);
        assert_eq!(scene.entities[0].frame_id, "map");
        assert_eq!(scene.entities[0].lines.len(), 1);
        assert_eq!(scene.entities[0].lines[0].points.len(), 2);
    }
}
