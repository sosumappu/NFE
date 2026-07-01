//! Pure occupancy-grid mapping.
//!
//! Runtime owns the async worker/channel boundary; this module is deterministic
//! and single-threaded so it is easy to test and later offload.

use nfe_core::mapping::{
    DistanceField, LoopClosureReport, MapStatus, MappingInput, OccupancyGrid, SubmapSummary,
    TrackMap,
};
use nfe_core::params::Tunable;
use nfe_core::{wrap_angle, Point2, Pose2};

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct MapperParams {
    #[param(0.02..0.20, default = 0.05)]
    pub resolution_m: f32,
    #[param(2.0..30.0, default = 12.0)]
    pub width_m: f32,
    #[param(2.0..30.0, default = 12.0)]
    pub height_m: f32,
    #[tunable(skip)]
    pub origin_x_m: f32,
    #[tunable(skip)]
    pub origin_y_m: f32,
    #[param(0.05..3.0, default = 0.85)]
    pub log_odds_occupied: f32,
    #[param(0.01..1.0, default = 0.35)]
    pub log_odds_free: f32,
    #[param(0.5..10.0, default = 4.0)]
    pub log_odds_limit: f32,
    #[param(1.0..12.0, default = 6.0)]
    pub max_range_m: f32,
    #[param(0.05..3.0, default = 1.0)]
    pub distance_field_truncation_m: f32,
    #[param(0.05..5.0, default = 1.0)]
    pub submap_translation_m: f32,
    #[param(0.05..std::f32::consts::PI, default = 0.35)]
    pub submap_yaw_rad: f32,
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
            resolution_m: 0.05,
            width_m: 12.0,
            height_m: 12.0,
            origin_x_m: -6.0,
            origin_y_m: -6.0,
            log_odds_occupied: 0.85,
            log_odds_free: 0.35,
            log_odds_limit: 4.0,
            max_range_m: 6.0,
            distance_field_truncation_m: 1.0,
            submap_translation_m: 1.0,
            submap_yaw_rad: 0.35,
            loop_closure_radius_m: 0.35,
            loop_closure_residual_good_m: 0.15,
            loop_closure_overlap_good: 0.45,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeskewedPoint {
    pub sensor_pose: Pose2,
    pub point: Point2,
    pub timestamp_us: u64,
    pub dist_m: f32,
}

#[derive(Clone, Debug)]
pub struct OccupancyGridMapper {
    params: MapperParams,
    grid: OccupancyGrid,
    distance_field: DistanceField,
    submaps: Vec<OccupancySubmap>,
    map: TrackMap,
    start_pose: Option<Pose2>,
    previous_pose: Option<Pose2>,
    previous_timestamp_us: Option<u64>,
    status: MapStatus,
}

#[derive(Clone, Debug)]
struct OccupancySubmap {
    id: u64,
    start_pose: Pose2,
    grid: OccupancyGrid,
    revision: u64,
    scan_count: u64,
}

impl OccupancyGridMapper {
    pub fn new(params: MapperParams) -> Self {
        let grid = new_grid(&params);
        let distance_field = compute_distance_field(&grid, params.distance_field_truncation_m);
        let map = TrackMap {
            occupancy: Some(grid.clone()),
            distance_field: Some(distance_field.clone()),
            ..Default::default()
        };
        Self {
            params,
            grid,
            distance_field,
            submaps: Vec::new(),
            map,
            start_pose: None,
            previous_pose: None,
            previous_timestamp_us: None,
            status: MapStatus {
                enabled: true,
                ..Default::default()
            },
        }
    }

    pub fn integrate(&mut self, input: MappingInput) {
        self.status.submitted_scans = self.status.submitted_scans.saturating_add(1);
        self.start_pose.get_or_insert(input.pose);
        self.ensure_active_submap(input.pose);
        self.maybe_start_new_submap(input.pose);

        let start_pose = self.previous_pose.unwrap_or(input.pose);
        let start_timestamp_us = self.previous_timestamp_us.unwrap_or_else(|| {
            input
                .cloud
                .points
                .iter()
                .map(|p| p.timestamp_us)
                .min()
                .unwrap_or(input.timestamp_us)
        });
        let mut end_pose = input.pose;
        let mut deskewed = deskew_scan_to_world(
            &input.cloud,
            start_pose,
            end_pose,
            start_timestamp_us,
            input.timestamp_us,
        );
        let closure = self.compute_loop_closure(end_pose, &deskewed);
        if closure.detected {
            end_pose = self.apply_loop_closure_correction(end_pose);
            deskewed = deskew_scan_to_world(
                &input.cloud,
                start_pose,
                end_pose,
                start_timestamp_us,
                input.timestamp_us,
            );
        }

        let mut occupied_updates = 0usize;
        for point in &deskewed {
            if integrate_ray(&mut self.grid, &self.params, point) {
                occupied_updates = occupied_updates.saturating_add(1);
            }
        }
        if let Some(active) = self.submaps.last_mut() {
            let mut submap_updates = 0usize;
            for point in &deskewed {
                if integrate_ray(&mut active.grid, &self.params, point) {
                    submap_updates = submap_updates.saturating_add(1);
                }
            }
            if submap_updates > 0 {
                active.revision = active.revision.saturating_add(1);
            }
            active.scan_count = active.scan_count.saturating_add(1);
            self.status.active_submap_id = active.id;
        }

        if occupied_updates > 0 {
            self.distance_field =
                compute_distance_field(&self.grid, self.params.distance_field_truncation_m);
            self.map.occupancy = Some(self.grid.clone());
            self.map.distance_field = Some(self.distance_field.clone());
            self.map.revision = self.map.revision.saturating_add(1);
            self.status.latest_revision = self.map.revision;
        }
        self.sync_submap_summaries();
        self.status.processed_scans = self.status.processed_scans.saturating_add(1);
        self.status.loop_closure = closure;
        self.previous_pose = Some(end_pose);
        self.previous_timestamp_us = Some(input.timestamp_us);
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

    pub fn submap_count(&self) -> usize {
        self.submaps.len()
    }

    fn ensure_active_submap(&mut self, pose: Pose2) {
        if self.submaps.is_empty() {
            self.start_submap(pose);
        }
    }

    fn maybe_start_new_submap(&mut self, pose: Pose2) {
        let Some(active) = self.submaps.last() else {
            return;
        };
        if active.scan_count == 0 {
            return;
        }
        let translation = Point2::new(pose.x, pose.y)
            .dist(&Point2::new(active.start_pose.x, active.start_pose.y));
        let yaw = wrap_angle(pose.yaw - active.start_pose.yaw).abs();
        if translation >= self.params.submap_translation_m || yaw >= self.params.submap_yaw_rad {
            self.start_submap(pose);
        }
    }

    fn start_submap(&mut self, pose: Pose2) {
        let id = self.submaps.last().map_or(0, |s| s.id.saturating_add(1));
        self.submaps.push(OccupancySubmap {
            id,
            start_pose: pose,
            grid: new_submap_grid(&self.params, pose),
            revision: 0,
            scan_count: 0,
        });
        self.status.active_submap_id = id;
        self.status.submap_count = self.submaps.len() as u32;
        self.sync_submap_summaries();
    }

    fn sync_submap_summaries(&mut self) {
        let active_id = self.submaps.last().map(|s| s.id);
        self.map.submaps = self
            .submaps
            .iter()
            .map(|submap| SubmapSummary {
                id: submap.id,
                start_pose: submap.start_pose,
                revision: submap.revision,
                scan_count: submap.scan_count,
                active: Some(submap.id) == active_id,
            })
            .collect();
        self.status.submap_count = self.submaps.len() as u32;
    }

    fn apply_loop_closure_correction(&mut self, closure_pose: Pose2) -> Pose2 {
        let Some(start) = self.start_pose else {
            return closure_pose;
        };
        let dx = start.x - closure_pose.x;
        let dy = start.y - closure_pose.y;
        let dyaw = wrap_angle(start.yaw - closure_pose.yaw);
        let denom = self.submaps.len().saturating_sub(1).max(1) as f32;
        for (idx, submap) in self.submaps.iter_mut().enumerate() {
            let alpha = idx as f32 / denom;
            submap.start_pose.x += alpha * dx;
            submap.start_pose.y += alpha * dy;
            submap.start_pose.yaw = wrap_angle(submap.start_pose.yaw + alpha * dyaw);
            submap.revision = submap.revision.saturating_add(1);
        }
        Pose2::new(
            closure_pose.x + dx,
            closure_pose.y + dy,
            wrap_angle(closure_pose.yaw + dyaw),
        )
    }

    fn compute_loop_closure(&self, pose: Pose2, deskewed: &[DeskewedPoint]) -> LoopClosureReport {
        let Some(start) = self.start_pose else {
            return LoopClosureReport::default();
        };
        if Point2::new(pose.x, pose.y).dist(&Point2::new(start.x, start.y))
            > self.params.loop_closure_radius_m
        {
            return LoopClosureReport::default();
        }
        let Some(field) = self.map.distance_field.as_ref() else {
            return LoopClosureReport::default();
        };

        let mut matched = 0usize;
        let mut total = 0usize;
        let mut residual = 0.0f32;
        for point in deskewed {
            if !(0.0..=self.params.max_range_m).contains(&point.dist_m) {
                continue;
            }
            if let Some(dist) = distance_at(field, point.point) {
                total += 1;
                residual += dist;
                if dist <= self.params.resolution_m * 2.0 {
                    matched += 1;
                }
            }
        }
        if total == 0 {
            return LoopClosureReport::default();
        }
        let residual_m = residual / total as f32;
        let overlap = matched as f32 / total as f32;
        LoopClosureReport {
            detected: residual_m <= self.params.loop_closure_residual_good_m
                && overlap >= self.params.loop_closure_overlap_good,
            residual_m,
            overlap,
        }
    }
}

impl Default for OccupancyGridMapper {
    fn default() -> Self {
        Self::new(MapperParams::default())
    }
}

pub fn deskew_scan_to_world(
    cloud: &nfe_core::sensors::LidarCloud,
    start_pose: Pose2,
    end_pose: Pose2,
    start_timestamp_us: u64,
    end_timestamp_us: u64,
) -> Vec<DeskewedPoint> {
    cloud
        .points
        .iter()
        .map(|point| {
            let pose = interpolate_pose(
                start_pose,
                end_pose,
                start_timestamp_us,
                end_timestamp_us,
                point.timestamp_us,
            );
            DeskewedPoint {
                sensor_pose: pose,
                point: pose.transform_point(&point.point2()),
                timestamp_us: point.timestamp_us,
                dist_m: point.dist_m,
            }
        })
        .collect()
}

fn interpolate_pose(
    start_pose: Pose2,
    end_pose: Pose2,
    start_timestamp_us: u64,
    end_timestamp_us: u64,
    timestamp_us: u64,
) -> Pose2 {
    if end_timestamp_us <= start_timestamp_us {
        return end_pose;
    }
    let alpha = ((timestamp_us.saturating_sub(start_timestamp_us)) as f32
        / (end_timestamp_us - start_timestamp_us) as f32)
        .clamp(0.0, 1.0);
    let yaw_delta = wrap_angle(end_pose.yaw - start_pose.yaw);
    Pose2::new(
        start_pose.x + (end_pose.x - start_pose.x) * alpha,
        start_pose.y + (end_pose.y - start_pose.y) * alpha,
        wrap_angle(start_pose.yaw + yaw_delta * alpha),
    )
}

fn new_grid(params: &MapperParams) -> OccupancyGrid {
    let resolution = params.resolution_m.max(1.0e-3);
    let width = (params.width_m / resolution).ceil().max(1.0) as u32;
    let height = (params.height_m / resolution).ceil().max(1.0) as u32;
    OccupancyGrid {
        origin: Point2::new(params.origin_x_m, params.origin_y_m),
        resolution_m: resolution,
        width,
        height,
        cells: vec![0.0; width as usize * height as usize],
    }
}

fn new_submap_grid(params: &MapperParams, pose: Pose2) -> OccupancyGrid {
    let resolution = params.resolution_m.max(1.0e-3);
    let width = (params.width_m / resolution).ceil().max(1.0) as u32;
    let height = (params.height_m / resolution).ceil().max(1.0) as u32;
    OccupancyGrid {
        origin: Point2::new(
            pose.x - 0.5 * width as f32 * resolution,
            pose.y - 0.5 * height as f32 * resolution,
        ),
        resolution_m: resolution,
        width,
        height,
        cells: vec![0.0; width as usize * height as usize],
    }
}

fn integrate_ray(grid: &mut OccupancyGrid, params: &MapperParams, point: &DeskewedPoint) -> bool {
    if !(0.0..=params.max_range_m).contains(&point.dist_m) {
        return false;
    }
    let Some(start) = world_to_cell(grid, point.sensor_pose.x, point.sensor_pose.y) else {
        return false;
    };
    let Some(end) = world_to_cell(grid, point.point.x, point.point.y) else {
        return false;
    };
    let line = grid_line(start, end);
    for cell in line.iter().take(line.len().saturating_sub(1)) {
        add_log_odds(grid, *cell, -params.log_odds_free, params.log_odds_limit);
    }
    add_log_odds(grid, end, params.log_odds_occupied, params.log_odds_limit);
    true
}

fn add_log_odds(grid: &mut OccupancyGrid, cell: (i32, i32), delta: f32, limit: f32) {
    if let Some(idx) = grid_index(grid, cell) {
        grid.cells[idx] = (grid.cells[idx] + delta).clamp(-limit, limit);
    }
}

fn world_to_cell(grid: &OccupancyGrid, x: f32, y: f32) -> Option<(i32, i32)> {
    let ix = ((x - grid.origin.x) / grid.resolution_m).floor() as i32;
    let iy = ((y - grid.origin.y) / grid.resolution_m).floor() as i32;
    if ix < 0 || iy < 0 || ix >= grid.width as i32 || iy >= grid.height as i32 {
        None
    } else {
        Some((ix, iy))
    }
}

fn grid_index(grid: &OccupancyGrid, cell: (i32, i32)) -> Option<usize> {
    let (ix, iy) = cell;
    if ix < 0 || iy < 0 || ix >= grid.width as i32 || iy >= grid.height as i32 {
        None
    } else {
        Some(iy as usize * grid.width as usize + ix as usize)
    }
}

fn grid_line(start: (i32, i32), end: (i32, i32)) -> Vec<(i32, i32)> {
    let (mut x0, mut y0) = start;
    let (x1, y1) = end;
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    let mut out = Vec::new();

    loop {
        out.push((x0, y0));
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }

    out
}

pub fn compute_distance_field(grid: &OccupancyGrid, truncation_m: f32) -> DistanceField {
    let occupied: Vec<_> = grid
        .cells
        .iter()
        .enumerate()
        .filter_map(|(idx, value)| {
            if *value > 0.0 {
                Some((idx % grid.width as usize, idx / grid.width as usize))
            } else {
                None
            }
        })
        .collect();
    let trunc = truncation_m.max(grid.resolution_m);
    let trunc2 = trunc * trunc;
    let mut distances_m = vec![trunc; grid.cells.len()];

    if occupied.is_empty() {
        return DistanceField {
            origin: grid.origin,
            resolution_m: grid.resolution_m,
            width: grid.width,
            height: grid.height,
            distances_m,
        };
    }

    for y in 0..grid.height as usize {
        for x in 0..grid.width as usize {
            let mut best2 = trunc2;
            for &(ox, oy) in &occupied {
                let dx = (x as f32 - ox as f32) * grid.resolution_m;
                let dy = (y as f32 - oy as f32) * grid.resolution_m;
                best2 = best2.min(dx * dx + dy * dy);
            }
            distances_m[y * grid.width as usize + x] = best2.sqrt();
        }
    }

    DistanceField {
        origin: grid.origin,
        resolution_m: grid.resolution_m,
        width: grid.width,
        height: grid.height,
        distances_m,
    }
}

pub fn distance_at(field: &DistanceField, point: Point2) -> Option<f32> {
    let ix = ((point.x - field.origin.x) / field.resolution_m).floor() as i32;
    let iy = ((point.y - field.origin.y) / field.resolution_m).floor() as i32;
    if ix < 0 || iy < 0 || ix >= field.width as i32 || iy >= field.height as i32 {
        None
    } else {
        Some(field.distances_m[iy as usize * field.width as usize + ix as usize])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::sensors::{LidarCloud, LidarPoint};

    fn mapper_params() -> MapperParams {
        MapperParams {
            resolution_m: 0.1,
            width_m: 4.0,
            height_m: 4.0,
            origin_x_m: -2.0,
            origin_y_m: -2.0,
            ..Default::default()
        }
    }

    fn point_cloud(timestamp_us: u64) -> LidarCloud {
        LidarCloud {
            points: vec![LidarPoint::from_polar(1.0, 0.0, timestamp_us)],
            timestamp_us,
        }
    }

    #[test]
    fn integrates_scan_into_occupancy_grid() {
        let mut mapper = OccupancyGridMapper::new(mapper_params());

        mapper.integrate(MappingInput {
            cloud: point_cloud(1),
            pose: Pose2::default(),
            timestamp_us: 1,
        });

        let map = mapper.map();
        assert_eq!(map.revision, 1);
        assert!(map
            .occupancy
            .as_ref()
            .unwrap()
            .cells
            .iter()
            .any(|v| *v > 0.0));
        assert!(map.distance_field.is_some());
    }

    #[test]
    fn submap_boundary_triggers_on_translation_threshold() {
        let mut mapper = OccupancyGridMapper::new(MapperParams {
            submap_translation_m: 0.5,
            submap_yaw_rad: 3.0,
            ..mapper_params()
        });

        mapper.integrate(MappingInput {
            cloud: point_cloud(1),
            pose: Pose2::default(),
            timestamp_us: 1,
        });
        mapper.integrate(MappingInput {
            cloud: point_cloud(2),
            pose: Pose2::new(0.6, 0.0, 0.0),
            timestamp_us: 2,
        });

        let status = mapper.status();
        let map = mapper.map();
        assert_eq!(mapper.submap_count(), 2);
        assert_eq!(status.submap_count, 2);
        assert_eq!(status.active_submap_id, 1);
        assert_eq!(map.submaps.len(), 2);
        assert_eq!(map.submaps[0].scan_count, 1);
        assert_eq!(map.submaps[1].scan_count, 1);
        assert!(map.submaps[1].active);
    }

    fn landmark_cloud(timestamp_us: u64) -> LidarCloud {
        LidarCloud {
            points: vec![
                LidarPoint::from_polar(1.0, 0.0, timestamp_us),
                LidarPoint::from_polar(1.0, 0.25, timestamp_us),
                LidarPoint::from_polar(1.2, -0.35, timestamp_us),
                LidarPoint::from_polar(0.8, 0.6, timestamp_us),
            ],
            timestamp_us,
        }
    }

    #[test]
    fn deskew_corrects_constant_velocity_scan_distortion() {
        let cloud = LidarCloud {
            points: vec![
                LidarPoint {
                    x: 2.0,
                    y: 0.0,
                    dist_m: 2.0,
                    angle_rad: 0.0,
                    timestamp_us: 0,
                },
                LidarPoint {
                    x: 1.0,
                    y: 0.0,
                    dist_m: 1.0,
                    angle_rad: 0.0,
                    timestamp_us: 1_000_000,
                },
            ],
            timestamp_us: 1_000_000,
        };

        let deskewed = deskew_scan_to_world(
            &cloud,
            Pose2::default(),
            Pose2::new(1.0, 0.0, 0.0),
            0,
            1_000_000,
        );

        assert_eq!(deskewed.len(), 2);
        for point in deskewed {
            assert!((point.point.x - 2.0).abs() < 1.0e-6, "point={point:?}");
            assert!(point.point.y.abs() < 1.0e-6, "point={point:?}");
        }
    }

    #[test]
    fn synthetic_loop_closure_corrects_submap_pose_graph() {
        let mut mapper = OccupancyGridMapper::new(MapperParams {
            submap_translation_m: 0.5,
            submap_yaw_rad: 3.0,
            loop_closure_radius_m: 0.35,
            loop_closure_residual_good_m: 0.20,
            loop_closure_overlap_good: 0.75,
            ..mapper_params()
        });

        mapper.integrate(MappingInput {
            cloud: landmark_cloud(1),
            pose: Pose2::default(),
            timestamp_us: 1,
        });
        mapper.integrate(MappingInput {
            cloud: landmark_cloud(2),
            pose: Pose2::new(0.6, 0.0, 0.0),
            timestamp_us: 2,
        });
        mapper.integrate(MappingInput {
            cloud: landmark_cloud(3),
            pose: Pose2::new(0.10, 0.0, 0.0),
            timestamp_us: 3,
        });

        let status = mapper.status();
        let map = mapper.map();
        assert!(status.loop_closure.detected, "status={status:?}");
        assert_eq!(map.submaps.len(), 3);
        assert!(map.submaps[0].start_pose.x.abs() < 1.0e-6);
        assert!(
            (map.submaps[1].start_pose.x - 0.55).abs() < 1.0e-6,
            "submaps={:?}",
            map.submaps
        );
        assert!(
            map.submaps[2].start_pose.x.abs() < 1.0e-6,
            "submaps={:?}",
            map.submaps
        );
    }

    #[test]
    fn adversarial_bad_loop_closure_is_rejected() {
        let mut mapper = OccupancyGridMapper::new(MapperParams {
            submap_translation_m: 0.5,
            submap_yaw_rad: 3.0,
            loop_closure_radius_m: 0.35,
            loop_closure_residual_good_m: 0.05,
            loop_closure_overlap_good: 0.75,
            ..mapper_params()
        });
        let adversarial = LidarCloud {
            points: vec![
                LidarPoint::from_polar(1.8, 1.5, 3),
                LidarPoint::from_polar(1.7, 1.3, 3),
                LidarPoint::from_polar(1.9, -1.4, 3),
            ],
            timestamp_us: 3,
        };

        mapper.integrate(MappingInput {
            cloud: landmark_cloud(1),
            pose: Pose2::default(),
            timestamp_us: 1,
        });
        mapper.integrate(MappingInput {
            cloud: landmark_cloud(2),
            pose: Pose2::new(0.6, 0.0, 0.0),
            timestamp_us: 2,
        });
        mapper.integrate(MappingInput {
            cloud: adversarial,
            pose: Pose2::new(0.10, 0.0, 0.0),
            timestamp_us: 3,
        });

        let status = mapper.status();
        let map = mapper.map();
        assert!(!status.loop_closure.detected, "status={status:?}");
        assert_eq!(map.submaps.len(), 3);
        assert!(
            (map.submaps[2].start_pose.x - 0.10).abs() < 1.0e-6,
            "bad closure should not correct graph: {:?}",
            map.submaps
        );
    }
}
