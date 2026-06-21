use nfe_core::mapping::TrackMap;
use nfe_core::params::Tunable;
use nfe_core::raceline::{RaceLine, RaceLinePoint};
use nfe_core::{wrap_angle, Point2};

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct RaceLineSolverParams {
    #[param(0.05..1.0, default = 0.20)]
    pub bin_width_m: f32,
    #[param(0.1..10.0, default = 1.2)]
    pub v_max_ms: f32,
    #[param(0.01..10.0, default = 1.0)]
    pub curvature_slowdown: f32,
    #[param(int, 0..8, default = 2)]
    pub smoothing_passes: usize,
}

impl Default for RaceLineSolverParams {
    fn default() -> Self {
        Self {
            bin_width_m: 0.20,
            v_max_ms: 1.2,
            curvature_slowdown: 1.0,
            smoothing_passes: 2,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum RaceLineError {
    EmptyMap,
    InsufficientBoundaries,
}

/// Generate a centerline and speed profile from explicit wall boundaries.
///
/// This is a deterministic, minimum-curvature-lite initial implementation: it
/// bins wall endpoints by x, averages upper/lower wall positions into a center
/// point, smooths, estimates curvature, then assigns a speed profile. The QP
/// solver can replace this behind the same function signature later.
pub fn solve_min_curvature(
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
        let speed = params.v_max_ms / (1.0 + params.curvature_slowdown * curvature.abs());
        out.push(RaceLinePoint {
            p: center[i],
            yaw: wrap_angle(yaw),
            curvature,
            speed_ms: speed,
            s_m: s_acc,
        });
    }

    Ok(RaceLine {
        points: out,
        closed: map.complete,
        revision: map.revision,
    })
}

fn curvature(a: Point2, b: Point2, c: Point2) -> f32 {
    let ab = a.dist(&b);
    let bc = b.dist(&c);
    let ca = c.dist(&a);
    let denom = (ab * bc * ca).max(1e-6);
    let area2 = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs();
    2.0 * area2 / denom
}
