use nfe_core::mapping::{OccupancyGrid, TrackMap};
use nfe_core::params::Tunable;
use nfe_core::raceline::{RaceLine, RaceLinePoint};
use nfe_core::{wrap_angle, Point2};

use crate::raceline::grid::{BinarizeParams, BinaryMask, Connectivity};
use crate::raceline::min_curvature::{
    optimize_min_curvature, MinCurvatureParams, MinCurvaturePath,
};
use crate::raceline::reference::{reference_path_from_contour, ReferencePathParams};
use crate::raceline::savgol::SavitzkyGolayParams;
use crate::raceline::velocity::{compute_velocity_profile, VelocityProfileParams};
use crate::raceline::watershed::{extract_separatrix, WatershedParams};

pub use crate::raceline::min_curvature::MinCurvatureError;
pub use crate::raceline::qp::QpError;
pub use crate::raceline::reference::ReferencePathError;
pub use crate::raceline::savgol::SavitzkyGolayError;
pub use crate::raceline::velocity::VelocityProfileError;
pub use crate::raceline::watershed::WatershedError;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct RaceLineSolverParams {
    #[param(0.05..1.0, default = 0.20)]
    pub bin_width_m: f32,
    #[param(0.1..10.0, default = 6.32)]
    pub v_max_ms: f32,
    #[param(0.01..10.0, default = 1.0)]
    pub curvature_slowdown: f32,
    #[param(0.1..30.0, default = 8.0)]
    pub velocity_lateral_accel_limit_ms2: f32,
    #[param(0.0..20.0, default = 4.0)]
    pub velocity_accel_limit_ms2: f32,
    #[param(0.0..30.0, default = 8.0)]
    pub velocity_brake_limit_ms2: f32,
    #[param(0.00001..0.01, default = 0.0001)]
    pub velocity_curvature_epsilon_m_inv: f32,
    #[param(int, 1..16, default = 8)]
    pub velocity_closed_passes: usize,
    #[param(int, 0..8, default = 2)]
    pub smoothing_passes: usize,
    #[param(int, 0..4, default = 1)]
    pub occupancy_morph_radius_cells: usize,
    #[param(int, 3..21, default = 7)]
    pub reference_savgol_window_points: usize,
    #[param(int, 1..5, default = 3)]
    pub reference_savgol_polynomial_order: usize,
    /// Non-positive means half the occupancy-grid resolution.
    #[param(0.0..0.25, default = 0.0)]
    pub width_step_m: f32,
    #[param(0.5..10.0, default = 5.0)]
    pub max_width_m: f32,
    /// Wall clearance applied to both lateral bounds, in metres.
    #[param(0.0..0.5, default = 0.05)]
    pub clearance_m: f32,
    /// Non-positive disables the optional curvature bound.
    #[param(0.0..20.0, default = 0.0)]
    pub max_curvature_m_inv: f32,
    /// Initial numerical regularization for reference-line adherence.
    #[param(0.0..0.1, default = 0.0001)]
    pub regularization_weight_m_inv4: f32,
    /// Maximum adjacent lateral-offset slope `|alpha_i - alpha_j| / ds`.
    #[param(0.0..2.0, default = 0.05)]
    pub max_adjacent_offset_slope: f32,
    /// Initial iteration cap for the linearized QP loop.
    #[param(int, 1..40, default = 30)]
    #[serde(alias = "optimization_iterations")]
    pub max_iterations: usize,
    #[param(0.0001..0.1, default = 0.005)]
    pub convergence_tolerance_m_inv: f32,
    /// Initial damping schedule; these are starting guesses, not settled constants.
    #[param(0.05..1.0, default = 0.1)]
    pub damping_first: f32,
    #[param(0.05..1.0, default = 0.1)]
    pub damping_second: f32,
    #[param(0.05..1.0, default = 0.1)]
    pub damping_later: f32,
}

impl Default for RaceLineSolverParams {
    fn default() -> Self {
        Self {
            bin_width_m: 0.20,
            v_max_ms: 6.32,
            curvature_slowdown: 1.0,
            velocity_lateral_accel_limit_ms2: 8.0,
            velocity_accel_limit_ms2: 4.0,
            velocity_brake_limit_ms2: 8.0,
            velocity_curvature_epsilon_m_inv: 1.0e-4,
            velocity_closed_passes: 8,
            smoothing_passes: 2,
            occupancy_morph_radius_cells: 1,
            reference_savgol_window_points: 7,
            reference_savgol_polynomial_order: 3,
            width_step_m: 0.0,
            max_width_m: 5.0,
            clearance_m: 0.05,
            max_curvature_m_inv: 0.0,
            regularization_weight_m_inv4: 1.0e-4,
            max_adjacent_offset_slope: 0.05,
            max_iterations: 30,
            convergence_tolerance_m_inv: 0.005,
            damping_first: 0.1,
            damping_second: 0.1,
            damping_later: 0.1,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum RaceLineError {
    EmptyMap,
    InsufficientBoundaries,
    InvalidOccupancyGrid {
        width: u32,
        height: u32,
        cells: usize,
        resolution_m: f32,
    },
    NonFiniteOccupancyCell {
        index: usize,
        value: f32,
    },
    Watershed(WatershedError),
    ReferencePath(ReferencePathError),
    Optimization(MinCurvatureError),
    VelocityProfile(VelocityProfileError),
}

impl From<WatershedError> for RaceLineError {
    fn from(value: WatershedError) -> Self {
        Self::Watershed(value)
    }
}

impl From<ReferencePathError> for RaceLineError {
    fn from(value: ReferencePathError) -> Self {
        Self::ReferencePath(value)
    }
}

impl From<MinCurvatureError> for RaceLineError {
    fn from(value: MinCurvatureError) -> Self {
        Self::Optimization(value)
    }
}

impl From<VelocityProfileError> for RaceLineError {
    fn from(value: VelocityProfileError) -> Self {
        Self::VelocityProfile(value)
    }
}

/// Generate a raceline from an occupancy grid when available.
///
/// Occupancy grids are the primary path. Boundary walls are retained only as a
/// legacy fallback for older fixtures and tools that do not provide occupancy.
pub fn solve_min_curvature(
    map: &TrackMap,
    params: &RaceLineSolverParams,
) -> Result<RaceLine, RaceLineError> {
    if let Some(grid) = &map.occupancy {
        return solve_from_occupancy(map, grid, params);
    }
    solve_from_boundaries_legacy(map, params)
}

fn solve_from_occupancy(
    map: &TrackMap,
    grid: &OccupancyGrid,
    params: &RaceLineSolverParams,
) -> Result<RaceLine, RaceLineError> {
    validate_occupancy_grid(grid)?;

    let mut mask = BinaryMask::from_occupancy(grid, BinarizeParams::default());
    let morph_radius = params.occupancy_morph_radius_cells;
    if morph_radius > 0 {
        mask = mask.close(morph_radius).open(morph_radius);
    }
    mask = mask.retain_largest_free_component(Connectivity::Four);

    let contour = extract_separatrix(&mask, WatershedParams::default())?;
    let reference = reference_path_from_contour(
        &mask,
        &contour,
        ReferencePathParams {
            smoothing: SavitzkyGolayParams {
                window_points: params.reference_savgol_window_points,
                polynomial_order: params.reference_savgol_polynomial_order,
            },
            width_step_m: params.width_step_m,
            max_width_m: params.max_width_m,
        },
    )?;
    let optimized = optimize_min_curvature(&reference, min_curvature_params(params))?;

    raceline_from_min_curvature_path(&optimized.path, map.revision, params)
}

fn validate_occupancy_grid(grid: &OccupancyGrid) -> Result<(), RaceLineError> {
    let expected = (grid.width as usize).saturating_mul(grid.height as usize);
    if grid.width == 0
        || grid.height == 0
        || expected != grid.cells.len()
        || !grid.resolution_m.is_finite()
        || grid.resolution_m <= 0.0
    {
        return Err(RaceLineError::InvalidOccupancyGrid {
            width: grid.width,
            height: grid.height,
            cells: grid.cells.len(),
            resolution_m: grid.resolution_m,
        });
    }
    for (index, value) in grid.cells.iter().copied().enumerate() {
        if !value.is_finite() {
            return Err(RaceLineError::NonFiniteOccupancyCell { index, value });
        }
    }
    Ok(())
}

fn min_curvature_params(params: &RaceLineSolverParams) -> MinCurvatureParams {
    MinCurvatureParams {
        clearance_m: params.clearance_m,
        max_curvature_m_inv: params.max_curvature_m_inv,
        regularization_weight_m_inv4: params.regularization_weight_m_inv4,
        max_adjacent_offset_slope: params.max_adjacent_offset_slope,
        max_iterations: params.max_iterations,
        convergence_tolerance_m_inv: params.convergence_tolerance_m_inv,
        damping_first: params.damping_first,
        damping_second: params.damping_second,
        damping_later: params.damping_later,
        ..MinCurvatureParams::default()
    }
}

fn raceline_from_min_curvature_path(
    path: &MinCurvaturePath,
    revision: u64,
    params: &RaceLineSolverParams,
) -> Result<RaceLine, RaceLineError> {
    let curvatures: Vec<_> = path
        .samples
        .iter()
        .map(|sample| sample.curvature_m_inv)
        .collect();
    let segment_lengths = path_segment_lengths(path);
    let velocity = compute_velocity_profile(
        &curvatures,
        &segment_lengths,
        path.closed,
        &velocity_profile_params(params),
    )?;
    let points = path
        .samples
        .iter()
        .enumerate()
        .map(|(index, sample)| RaceLinePoint {
            p: sample.position,
            yaw: wrap_angle(sample.tangent.y.atan2(sample.tangent.x)),
            curvature: sample.curvature_m_inv,
            speed_ms: velocity.speed_ms[index],
            accel_x_ms2: velocity.accel_x_ms2[index],
            s_m: sample.s_m,
        })
        .collect();

    Ok(RaceLine {
        points,
        closed: path.closed,
        revision,
    })
}

fn velocity_profile_params(params: &RaceLineSolverParams) -> VelocityProfileParams {
    VelocityProfileParams {
        top_speed_ms: params.v_max_ms,
        lateral_accel_limit_ms2: params.velocity_lateral_accel_limit_ms2,
        accel_limit_ms2: params.velocity_accel_limit_ms2,
        brake_limit_ms2: params.velocity_brake_limit_ms2,
        curvature_epsilon_m_inv: params.velocity_curvature_epsilon_m_inv,
        closed_passes: params.velocity_closed_passes,
    }
}

fn path_segment_lengths(path: &MinCurvaturePath) -> Vec<f32> {
    let mut lengths = Vec::with_capacity(if path.closed {
        path.samples.len()
    } else {
        path.samples.len().saturating_sub(1)
    });
    for i in 0..path.samples.len().saturating_sub(1) {
        lengths.push(path.samples[i].position.dist(&path.samples[i + 1].position));
    }
    if path.closed && !path.samples.is_empty() {
        lengths.push(
            path.samples[path.samples.len() - 1]
                .position
                .dist(&path.samples[0].position),
        );
    }
    lengths
}

fn solve_from_boundaries_legacy(
    map: &TrackMap,
    params: &RaceLineSolverParams,
) -> Result<RaceLine, RaceLineError> {
    if map.boundaries.walls.is_empty() {
        return Err(RaceLineError::EmptyMap);
    }
    let mut pts = Vec::new();
    for w in &map.boundaries.walls {
        pts.push(w.p0);
        pts.push(w.p1);
    }
    let min_x = pts.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
    let max_x = pts.iter().map(|p| p.x).fold(f32::NEG_INFINITY, f32::max);
    if !min_x.is_finite() || max_x <= min_x {
        return Err(RaceLineError::InsufficientBoundaries);
    }

    let bins = (((max_x - min_x) / params.bin_width_m).ceil() as usize).max(2);
    let mut top: Vec<Vec<f32>> = vec![Vec::new(); bins];
    let mut bot: Vec<Vec<f32>> = vec![Vec::new(); bins];
    for p in pts {
        let idx = (((p.x - min_x) / params.bin_width_m).floor() as usize).min(bins - 1);
        if p.y >= 0.0 {
            top[idx].push(p.y);
        } else {
            bot[idx].push(p.y);
        }
    }

    let mut center = Vec::new();
    for i in 0..bins {
        if top[i].is_empty() || bot[i].is_empty() {
            continue;
        }
        let x = min_x + (i as f32 + 0.5) * params.bin_width_m;
        let y_top = top[i].iter().sum::<f32>() / top[i].len() as f32;
        let y_bot = bot[i].iter().sum::<f32>() / bot[i].len() as f32;
        center.push(Point2::new(x, 0.5 * (y_top + y_bot)));
    }
    if center.len() < 3 {
        return Err(RaceLineError::InsufficientBoundaries);
    }

    for _ in 0..params.smoothing_passes {
        let mut next = center.clone();
        for i in 1..center.len() - 1 {
            next[i].x = 0.25 * center[i - 1].x + 0.5 * center[i].x + 0.25 * center[i + 1].x;
            next[i].y = 0.25 * center[i - 1].y + 0.5 * center[i].y + 0.25 * center[i + 1].y;
        }
        center = next;
    }

    let mut out = Vec::with_capacity(center.len());
    let mut s_acc = 0.0;
    for i in 0..center.len() {
        if i > 0 {
            s_acc += center[i].dist(&center[i - 1]);
        }
        let prev = center[i.saturating_sub(1)];
        let next = center[(i + 1).min(center.len() - 1)];
        let yaw = (next.y - prev.y).atan2(next.x - prev.x);
        let curvature = curvature(prev, center[i], next);
        out.push(RaceLinePoint {
            p: center[i],
            yaw: wrap_angle(yaw),
            curvature,
            speed_ms: placeholder_speed(curvature, params),
            accel_x_ms2: 0.0,
            s_m: s_acc,
        });
    }

    Ok(RaceLine {
        points: out,
        closed: map.complete,
        revision: map.revision,
    })
}

fn placeholder_speed(curvature_m_inv: f32, params: &RaceLineSolverParams) -> f32 {
    let v_max = params.v_max_ms.max(0.0);
    let slowdown = params.curvature_slowdown.max(0.0);
    let denom = 1.0 + slowdown * curvature_m_inv.abs();
    if denom.is_finite() && denom > 0.0 {
        v_max / denom
    } else {
        0.0
    }
}

fn curvature(a: Point2, b: Point2, c: Point2) -> f32 {
    let ab = a.dist(&b);
    let bc = b.dist(&c);
    let ca = c.dist(&a);
    let denom = (ab * bc * ca).max(1e-6);
    let area2 = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs();
    2.0 * area2 / denom
}
