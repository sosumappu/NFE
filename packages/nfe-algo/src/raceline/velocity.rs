#![allow(dead_code)]

use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct VelocityProfileParams {
    #[param(0.1..20.0, default = 6.32)]
    pub top_speed_ms: f32,
    #[param(0.1..30.0, default = 8.0)]
    pub lateral_accel_limit_ms2: f32,
    #[param(0.0..20.0, default = 4.0)]
    pub accel_limit_ms2: f32,
    #[param(0.0..30.0, default = 8.0)]
    pub brake_limit_ms2: f32,
    #[param(0.00001..0.01, default = 0.0001)]
    pub curvature_epsilon_m_inv: f32,
    #[param(int, 1..16, default = 8)]
    pub closed_passes: usize,
}

impl Default for VelocityProfileParams {
    fn default() -> Self {
        Self {
            top_speed_ms: 6.32,
            lateral_accel_limit_ms2: 8.0,
            accel_limit_ms2: 4.0,
            brake_limit_ms2: 8.0,
            curvature_epsilon_m_inv: 1.0e-4,
            closed_passes: 8,
        }
    }
}

const CONVERGENCE_TOLERANCE_MS: f32 = 1.0e-4;

#[derive(Clone, Debug, PartialEq)]
pub struct VelocityProfile {
    pub speed_ms: Vec<f32>,
    pub accel_x_ms2: Vec<f32>,
    pub diagnostics: VelocityProfileDiagnostics,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityProfileDiagnostics {
    pub passes_run: usize,
    pub converged: bool,
    pub max_delta_ms: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum VelocityProfileError {
    TooFewSamples,
    DimensionMismatch,
    NonFiniteInput,
    NonPositiveSegment { index: usize, length_m: f32 },
    InvalidParameters,
}

pub fn compute_velocity_profile(
    curvature_m_inv: &[f32],
    segment_lengths_m: &[f32],
    closed: bool,
    params: &VelocityProfileParams,
) -> Result<VelocityProfile, VelocityProfileError> {
    validate_params(params)?;
    let n = curvature_m_inv.len();
    if n < 2 {
        return Err(VelocityProfileError::TooFewSamples);
    }
    let expected_segments = if closed { n } else { n - 1 };
    if segment_lengths_m.len() != expected_segments {
        return Err(VelocityProfileError::DimensionMismatch);
    }
    for (index, curvature) in curvature_m_inv.iter().copied().enumerate() {
        if !curvature.is_finite() {
            return Err(VelocityProfileError::NonFiniteInput);
        }
        if index < segment_lengths_m.len() {
            let length = segment_lengths_m[index];
            if !length.is_finite() {
                return Err(VelocityProfileError::NonFiniteInput);
            }
            if length <= 0.0 {
                return Err(VelocityProfileError::NonPositiveSegment {
                    index,
                    length_m: length,
                });
            }
        }
    }

    let top_speed = params.top_speed_ms.max(0.0);
    let mut speed: Vec<_> = curvature_m_inv
        .iter()
        .map(|curvature| corner_speed_cap(*curvature, params).min(top_speed))
        .collect();

    let max_passes = if closed {
        params.closed_passes.max(1)
    } else {
        1
    };
    let mut passes_run = 0;
    let mut converged = !closed;
    let mut max_delta_ms = 0.0;
    for pass in 0..max_passes {
        let before = speed.clone();
        forward_pass(
            &mut speed,
            segment_lengths_m,
            closed,
            params.accel_limit_ms2.max(0.0),
        );
        backward_pass(
            &mut speed,
            segment_lengths_m,
            closed,
            params.brake_limit_ms2.max(0.0),
        );
        passes_run = pass + 1;
        max_delta_ms = max_abs_delta(&before, &speed);
        if !closed || max_delta_ms <= CONVERGENCE_TOLERANCE_MS {
            converged = true;
            break;
        }
    }

    let accel_x_ms2 = accelerations(&speed, segment_lengths_m, closed);
    Ok(VelocityProfile {
        speed_ms: speed,
        accel_x_ms2,
        diagnostics: VelocityProfileDiagnostics {
            passes_run,
            converged,
            max_delta_ms,
        },
    })
}

fn validate_params(params: &VelocityProfileParams) -> Result<(), VelocityProfileError> {
    if !params.top_speed_ms.is_finite()
        || params.top_speed_ms < 0.0
        || !params.lateral_accel_limit_ms2.is_finite()
        || params.lateral_accel_limit_ms2 < 0.0
        || !params.accel_limit_ms2.is_finite()
        || params.accel_limit_ms2 < 0.0
        || !params.brake_limit_ms2.is_finite()
        || params.brake_limit_ms2 < 0.0
        || !params.curvature_epsilon_m_inv.is_finite()
        || params.curvature_epsilon_m_inv <= 0.0
    {
        return Err(VelocityProfileError::InvalidParameters);
    }
    Ok(())
}

fn corner_speed_cap(curvature_m_inv: f32, params: &VelocityProfileParams) -> f32 {
    let curvature = curvature_m_inv.abs();
    if curvature <= params.curvature_epsilon_m_inv || params.lateral_accel_limit_ms2 <= 0.0 {
        return params.top_speed_ms.max(0.0);
    }
    (params.lateral_accel_limit_ms2 / curvature).sqrt()
}

fn forward_pass(speed: &mut [f32], segment_lengths_m: &[f32], closed: bool, accel_limit_ms2: f32) {
    if accel_limit_ms2 <= 0.0 {
        return;
    }
    let n = speed.len();
    let edge_count = if closed { n } else { n - 1 };
    for (edge, segment_length_m) in segment_lengths_m
        .iter()
        .copied()
        .enumerate()
        .take(edge_count)
    {
        let i = edge;
        let j = if i + 1 == n { 0 } else { i + 1 };
        let cap_sq = speed[i] * speed[i] + 2.0 * accel_limit_ms2 * segment_length_m;
        let cap = cap_sq.max(0.0).sqrt();
        if speed[j] > cap {
            speed[j] = cap;
        }
    }
}

fn backward_pass(speed: &mut [f32], segment_lengths_m: &[f32], closed: bool, brake_limit_ms2: f32) {
    if brake_limit_ms2 <= 0.0 {
        return;
    }
    let n = speed.len();
    let edge_count = if closed { n } else { n - 1 };
    for (edge, segment_length_m) in segment_lengths_m
        .iter()
        .copied()
        .enumerate()
        .take(edge_count)
        .rev()
    {
        let i = edge;
        let j = if i + 1 == n { 0 } else { i + 1 };
        let cap_sq = speed[j] * speed[j] + 2.0 * brake_limit_ms2 * segment_length_m;
        let cap = cap_sq.max(0.0).sqrt();
        if speed[i] > cap {
            speed[i] = cap;
        }
    }
}

fn max_abs_delta(before: &[f32], after: &[f32]) -> f32 {
    before
        .iter()
        .zip(after)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f32::max)
}

fn accelerations(speed: &[f32], segment_lengths_m: &[f32], closed: bool) -> Vec<f32> {
    let n = speed.len();
    let mut accel = vec![0.0; n];
    let edge_count = if closed { n } else { n - 1 };
    for (edge, segment_length_m) in segment_lengths_m
        .iter()
        .copied()
        .enumerate()
        .take(edge_count)
    {
        let i = edge;
        let j = if i + 1 == n { 0 } else { i + 1 };
        accel[i] = (speed[j] * speed[j] - speed[i] * speed[i]) / (2.0 * segment_length_m);
    }
    accel
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> VelocityProfileParams {
        VelocityProfileParams {
            top_speed_ms: 10.0,
            lateral_accel_limit_ms2: 4.0,
            accel_limit_ms2: 1.0,
            brake_limit_ms2: 2.0,
            closed_passes: 4,
            ..VelocityProfileParams::default()
        }
    }

    #[test]
    fn circular_curve_is_lateral_accel_limited() {
        let profile = compute_velocity_profile(&[1.0; 8], &[1.0; 8], true, &params()).unwrap();

        for speed in profile.speed_ms {
            assert!((speed - 2.0).abs() < 1.0e-5, "speed={speed}");
        }
        assert!(profile.diagnostics.converged);
        assert_eq!(profile.diagnostics.passes_run, 1);
    }

    #[test]
    fn forward_pass_limits_acceleration() {
        let profile =
            compute_velocity_profile(&[4.0, 0.0, 0.0], &[1.0, 1.0], false, &params()).unwrap();

        assert!((profile.speed_ms[0] - 1.0).abs() < 1.0e-5);
        assert!(profile.speed_ms[1] <= 3.0_f32.sqrt() + 1.0e-5);
        assert!(profile.accel_x_ms2[0] <= 1.0 + 1.0e-5);
    }

    #[test]
    fn backward_pass_starts_braking_before_corner() {
        let profile =
            compute_velocity_profile(&[0.0, 0.0, 4.0], &[1.0, 1.0], false, &params()).unwrap();

        assert!(profile.speed_ms[1] < params().top_speed_ms);
        assert!(profile.speed_ms[1] <= (1.0_f32 + 2.0 * 2.0 * 1.0).sqrt() + 1.0e-5);
        assert!(profile.accel_x_ms2[1] < 0.0);
    }
}
