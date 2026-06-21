//! Raceline data model.

use crate::Point2;

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct RaceLinePoint {
    pub p: Point2,
    pub yaw: f32,
    pub curvature: f32,
    pub speed_ms: f32,
    pub s_m: f32,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct RaceLine {
    pub points: Vec<RaceLinePoint>,
    pub closed: bool,
    pub revision: u64,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct RaceReference {
    pub target: RaceLinePoint,
    pub lateral_error_m: f32,
    pub heading_error_rad: f32,
    pub lookahead_m: f32,
    pub confidence: f32,
}
