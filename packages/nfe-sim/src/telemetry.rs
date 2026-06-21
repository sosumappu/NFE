use nfe_core::telemetry::{
    GroundTruthStateTelemetry, GroundTruthTelemetry, TelemetryEvent, WallKind,
    WallSegmentTelemetry, WorldSnapshotTelemetry, WorldTelemetry,
};
use nfe_core::{Point2, Pose2};

use crate::model::{ControlCommand, VehicleState};
use crate::world::World;

pub fn world_snapshot_event(world: &World, timestamp_us: u64) -> TelemetryEvent {
    TelemetryEvent::World(WorldTelemetry::Snapshot(WorldSnapshotTelemetry {
        timestamp_us,
        frame_id: "map".to_string(),
        revision: 0,
        walls: world_segments(world),
        start_pose: Pose2 {
            x: world.start.x,
            y: world.start.y,
            yaw: world.start.yaw_rad,
        },
        waypoints: world
            .waypoints
            .iter()
            .map(|(x, y)| Point2 { x: *x, y: *y })
            .collect(),
    }))
}

pub fn ground_truth_event(
    state: VehicleState,
    command: ControlCommand,
    timestamp_us: u64,
) -> TelemetryEvent {
    TelemetryEvent::GroundTruth(GroundTruthTelemetry::State(GroundTruthStateTelemetry {
        timestamp_us,
        frame_id: "map".to_string(),
        pose: Pose2 {
            x: state.x,
            y: state.y,
            yaw: state.yaw_rad,
        },
        vx_ms: state.vx,
        vy_ms: state.vy,
        yaw_rate_rad_s: state.yaw_rate,
        steering_rad: command.steering_rad,
        throttle: command.throttle,
    }))
}

fn world_segments(world: &World) -> Vec<WallSegmentTelemetry> {
    let mut out = Vec::new();
    let mut id = 0u64;
    for seg in world.inner_segments() {
        out.push(segment(id, WallKind::Inner, seg));
        id += 1;
    }
    for seg in world.outer_segments() {
        out.push(segment(id, WallKind::Outer, seg));
        id += 1;
    }
    out
}

fn segment(id: u64, kind: WallKind, seg: crate::world::Seg) -> WallSegmentTelemetry {
    WallSegmentTelemetry {
        id,
        kind,
        p0: Point2 {
            x: seg.ax,
            y: seg.ay,
        },
        p1: Point2 {
            x: seg.bx,
            y: seg.by,
        },
    }
}
