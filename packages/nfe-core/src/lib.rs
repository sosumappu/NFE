//! `nfe-core` — shared types, the `Tunable` params registry, and the geometry
//! primitives used across perception, estimation, mapping, and control. No I/O,
//! no tokio, no hardware. This is the crate the tuner and unit tests link
//! without pulling in the runtime.

// The `#[derive(Tunable)]` macro emits paths rooted at `::nfe_core`. Inside
// this crate that name does not exist by default; alias self so generated code
// resolves identically here and in downstream crates.
extern crate self as nfe_core;

pub mod control;
pub mod estimation;
pub mod io;
pub mod localization;
pub mod mapping;
pub mod params;
pub mod raceline;
pub mod sensors;
pub mod telemetry;

use serde::{Deserialize, Serialize};

/// 2D point in some frame (car-local or world, context-dependent).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

impl Point2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
    pub fn dist(&self, o: &Point2) -> f32 {
        (self.x - o.x).hypot(self.y - o.y)
    }
}

/// Planar pose: position + heading. World frame unless stated otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose2 {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

impl Pose2 {
    pub const fn new(x: f32, y: f32, yaw: f32) -> Self {
        Self { x, y, yaw }
    }

    /// Transform a car-local point into the world frame using this pose.
    pub fn transform_point(&self, p: &Point2) -> Point2 {
        let (s, c) = self.yaw.sin_cos();
        Point2 {
            x: self.x + c * p.x - s * p.y,
            y: self.y + s * p.x + c * p.y,
        }
    }
}

/// Wrap an angle to (-pi, pi].
#[inline]
pub fn wrap_angle(a: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    (a + PI).rem_euclid(TAU) - PI
}

/// What controllers actually consume regardless of estimator/mode.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct MotionState {
    pub speed_ms: f32,
    pub yaw_rate_rad_s: f32,
}

/// A fitted wall as an infinite line in normal form: n·p = d, with unit normal
/// (nx, ny) and signed offset d. Endpoints bound the supporting segment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WallLine {
    pub nx: f32,
    pub ny: f32,
    pub d: f32,
    pub p0: Point2,
    pub p1: Point2,
    /// Fraction of candidate points that supported this line in [0,1].
    pub support: f32,
}

impl WallLine {
    /// Signed perpendicular distance from a point to the infinite line.
    pub fn signed_distance(&self, p: &Point2) -> f32 {
        self.nx * p.x + self.ny * p.y - self.d
    }
}
