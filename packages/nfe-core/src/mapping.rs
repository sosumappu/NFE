//! Mapping trait boundary.
//!
//! The runtime may implement this in-process over channels, in another thread,
//! or later as an RPC/coprocessor boundary. Callers submit immutable scan+pose
//! work packets and poll snapshots/status; no mapper implementation is allowed
//! to block the 100 Hz control loop.

use crate::sensors::LidarCloud;
use crate::{Pose2, WallLine};

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct BoundarySet {
    pub walls: Vec<WallLine>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct OccupancyGrid {
    pub origin: crate::Point2,
    pub resolution_m: f32,
    pub width: u32,
    pub height: u32,
    /// Row-major log-odds occupancy values. Positive means occupied, negative
    /// means observed free space, and zero means unknown.
    pub cells: Vec<f32>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DistanceField {
    pub origin: crate::Point2,
    pub resolution_m: f32,
    pub width: u32,
    pub height: u32,
    /// Row-major distance-to-nearest-occupied-cell values in metres.
    pub distances_m: Vec<f32>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SubmapSummary {
    pub id: u64,
    pub start_pose: Pose2,
    pub revision: u64,
    pub scan_count: u64,
    pub active: bool,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct TrackMap {
    pub boundaries: BoundarySet,
    pub occupancy: Option<OccupancyGrid>,
    pub distance_field: Option<DistanceField>,
    pub submaps: Vec<SubmapSummary>,
    pub complete: bool,
    pub revision: u64,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct LoopClosureReport {
    pub detected: bool,
    pub residual_m: f32,
    pub overlap: f32,
}

impl Default for LoopClosureReport {
    fn default() -> Self {
        Self {
            detected: false,
            residual_m: f32::INFINITY,
            overlap: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct MapStatus {
    pub enabled: bool,
    pub submitted_scans: u64,
    pub processed_scans: u64,
    pub dropped_scans: u64,
    pub latest_revision: u64,
    pub active_submap_id: u64,
    pub submap_count: u32,
    pub loop_closure: LoopClosureReport,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MappingInput {
    pub cloud: LidarCloud,
    pub pose: Pose2,
    pub timestamp_us: u64,
}

pub trait MapperClient {
    /// Submit a scan for asynchronous mapping. Must be non-blocking; returns
    /// false if disabled or if the work queue is full and the scan was dropped.
    fn submit(&mut self, input: MappingInput) -> bool;
    fn latest_status(&self) -> MapStatus;
    fn latest_map(&self) -> Option<TrackMap>;
}
