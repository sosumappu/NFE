//! Pure mapping algorithm: RANSAC wall extraction + explicit wall accumulation.
//!
//! Runtime owns the async worker/channel boundary; this module is deterministic
//! and single-threaded so it is easy to test and later offload.

use nfe_core::mapping::{BoundarySet, LoopClosureReport, MapStatus, MappingInput, TrackMap};
use nfe_core::params::Tunable;
use nfe_core::{Point2, Pose2, WallLine};

use crate::perception::ransac::{fit_walls, RansacParams};

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct MapperParams {
    #[tunable(nested)]
    pub ransac: RansacParams,
    #[param(0.05..1.0, default = 0.20)]
    pub min_wall_length_m: f32,
    #[param(0.05..2.0, default = 0.35)]
    pub loop_closure_radius_m: f32,
    #[param(0.02..1.0, default = 0.15)]
    pub loop_closure_residual_good_m: f32,
    #[param(0.1..0.95, default = 0.45)]
    pub loop_closure_overlap_good: f32,
}

impl Default for MapperParams {
    fn default() -> Self {
        Self {
            ransac: RansacParams::default(),
            min_wall_length_m: 0.20,
            loop_closure_radius_m: 0.35,
            loop_closure_residual_good_m: 0.15,
            loop_closure_overlap_good: 0.45,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RansacWallMapper {
    params: MapperParams,
    map: TrackMap,
    start_pose: Option<Pose2>,
    status: MapStatus,
    seed: u64,
}

impl RansacWallMapper {
    pub fn new(params: MapperParams, seed: u64) -> Self {
        Self {
            params,
            map: TrackMap::default(),
            start_pose: None,
            status: MapStatus {
                enabled: true,
                ..Default::default()
            },
            seed,
        }
    }

    pub fn integrate(&mut self, input: MappingInput) {
        self.status.submitted_scans = self.status.submitted_scans.saturating_add(1);
        self.start_pose.get_or_insert(input.pose);
        let points = input.cloud.as_points2();
        let mut local = fit_walls(&points, &self.params.ransac, self.seed ^ input.timestamp_us);
        local.retain(|w| w.p0.dist(&w.p1) >= self.params.min_wall_length_m);
        for w in &local {
            self.map
                .boundaries
                .walls
                .push(transform_wall(input.pose, w));
        }
        if !local.is_empty() {
            self.map.revision = self.map.revision.saturating_add(1);
            self.status.latest_revision = self.map.revision;
        }
        self.status.processed_scans = self.status.processed_scans.saturating_add(1);
        self.status.loop_closure = self.compute_loop_closure(input.pose, &local);
    }

    /// Physical start-line crossing finalizes map completeness; geometric loop
    /// closure is only quality/consistency and remains in status.
    pub fn mark_lap_complete(&mut self) {
        self.map.complete = true;
    }

    pub fn map(&self) -> TrackMap {
        self.map.clone()
    }

    pub fn status(&self) -> MapStatus {
        self.status
    }

    fn compute_loop_closure(&self, pose: Pose2, local_walls: &[WallLine]) -> LoopClosureReport {
        let Some(start) = self.start_pose else {
            return LoopClosureReport::default();
        };
        if Point2::new(pose.x, pose.y).dist(&Point2::new(start.x, start.y))
            > self.params.loop_closure_radius_m
        {
            return LoopClosureReport::default();
        }
        if self.map.boundaries.walls.is_empty() || local_walls.is_empty() {
            return LoopClosureReport::default();
        }
        let world: Vec<_> = local_walls
            .iter()
            .map(|w| transform_wall(pose, w))
            .collect();
        let mut matches = 0usize;
        let mut residual = 0.0f32;
        for w in &world {
            let mid = midpoint(w);
            let best = self
                .map
                .boundaries
                .walls
                .iter()
                .map(|mw| mw.signed_distance(&mid).abs())
                .fold(f32::INFINITY, f32::min);
            if best <= self.params.loop_closure_residual_good_m {
                matches += 1;
                residual += best;
            }
        }
        if matches == 0 {
            return LoopClosureReport {
                detected: true,
                residual_m: f32::INFINITY,
                overlap: 0.0,
            };
        }
        LoopClosureReport {
            detected: true,
            residual_m: residual / matches as f32,
            overlap: matches as f32 / world.len() as f32,
        }
    }
}

fn midpoint(w: &WallLine) -> Point2 {
    Point2::new(0.5 * (w.p0.x + w.p1.x), 0.5 * (w.p0.y + w.p1.y))
}

fn transform_wall(pose: Pose2, w: &WallLine) -> WallLine {
    let p0 = pose.transform_point(&w.p0);
    let p1 = pose.transform_point(&w.p1);
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let len = dx.hypot(dy).max(1e-6);
    let nx = -dy / len;
    let ny = dx / len;
    let d = nx * p0.x + ny * p0.y;
    WallLine {
        nx,
        ny,
        d,
        p0,
        p1,
        support: w.support,
    }
}

impl From<RansacWallMapper> for TrackMap {
    fn from(value: RansacWallMapper) -> Self {
        value.map
    }
}

impl Default for RansacWallMapper {
    fn default() -> Self {
        Self::new(MapperParams::default(), 0)
    }
}

#[allow(dead_code)]
fn _boundary_set(walls: Vec<WallLine>) -> BoundarySet {
    BoundarySet { walls }
}
