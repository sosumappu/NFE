#![allow(dead_code)]

use std::collections::BTreeMap;

use nfe_core::Point2;

use crate::raceline::geometry::UnitVector2;
use crate::raceline::qp::{solve_qp, LinearInequality, QpError, QpSettings, QuadraticProgram};
use crate::raceline::reference::ReferencePath;

const MIN_SEGMENT_LENGTH_M: f64 = 1.0e-4;

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MinCurvatureParams {
    /// Total wall clearance kept on each side of the vehicle, in metres.
    pub clearance_m: f32,
    /// Non-positive disables the optional steering-limit curvature bound.
    pub max_curvature_m_inv: f32,
    /// Weight for the reference-adherence term `lambda * integral(alpha^2 ds)`.
    /// This is an initial numerical regularization value, not a tuned vehicle constant.
    pub regularization_weight_m_inv4: f32,
    /// Maximum adjacent lateral-offset slope `|alpha_i - alpha_j| / ds`.
    /// This preserves local path geometry by preventing neighboring offsets
    /// from diverging enough to collapse short segments.
    pub max_adjacent_offset_slope: f32,
    pub max_iterations: usize,
    pub convergence_tolerance_m_inv: f32,
    /// Initial damping values for the first, second, and later linearized QP solves.
    /// These are starting guesses that should be tuned against real maps.
    pub damping_first: f32,
    pub damping_second: f32,
    pub damping_later: f32,
    pub qp: QpSettings,
}

impl Default for MinCurvatureParams {
    fn default() -> Self {
        Self {
            clearance_m: 0.05,
            max_curvature_m_inv: 0.0,
            regularization_weight_m_inv4: 1.0e-4,
            max_adjacent_offset_slope: 0.05,
            max_iterations: 30,
            convergence_tolerance_m_inv: 0.005,
            damping_first: 0.1,
            damping_second: 0.1,
            damping_later: 0.1,
            qp: QpSettings::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MinCurvatureSample {
    pub position: Point2,
    pub tangent: UnitVector2,
    pub normal: UnitVector2,
    pub curvature_m_inv: f32,
    pub alpha_m: f32,
    pub s_m: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MinCurvaturePath {
    pub samples: Vec<MinCurvatureSample>,
    pub closed: bool,
    pub length_m: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MinCurvatureResult {
    pub path: MinCurvaturePath,
    pub offsets_m: Vec<f32>,
    pub iterations: usize,
    pub status: MinCurvatureStatus,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum MinCurvatureStatus {
    Converged { max_linearization_error_m_inv: f32 },
    ReferenceFallback { reason: MinCurvatureFallbackReason },
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum MinCurvatureFallbackReason {
    DidNotConverge {
        iterations: usize,
        max_linearization_error_m_inv: f32,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub enum MinCurvatureError {
    TooFewSamples,
    InvalidParameters,
    NonFiniteInput,
    InfeasibleBounds {
        index: usize,
        lower_m: f32,
        upper_m: f32,
        clearance_m: f32,
    },
    DegenerateGeometry {
        index: usize,
    },
    Qp(QpError),
}

pub(crate) fn optimize_min_curvature(
    reference: &ReferencePath,
    params: MinCurvatureParams,
) -> Result<MinCurvatureResult, MinCurvatureError> {
    validate_params(&params)?;
    if reference.samples.len() < 3 {
        return Err(MinCurvatureError::TooFewSamples);
    }

    let original_points: Vec<_> = reference
        .samples
        .iter()
        .map(|sample| sample.position)
        .collect();
    let normals: Vec<_> = reference
        .samples
        .iter()
        .map(|sample| sample.normal)
        .collect();
    validate_points(&original_points)?;
    let bounds = alpha_bounds(reference, params.clearance_m)?;
    let weights = segment_weights(&original_points, reference.closed)?;
    let edge_lengths = adjacent_edge_lengths(&original_points, reference.closed)?;

    let mut alpha = vec![0.0_f64; original_points.len()];
    let mut max_error = f64::INFINITY;
    for iteration in 0..params.max_iterations.max(1) {
        let current_points = apply_offsets(&original_points, &normals, &alpha);
        let linearization = curvature_linearization(
            &normals,
            &alpha,
            &current_points,
            reference.closed,
            &weights,
        )?;
        let problem = build_problem(&linearization, &bounds, &edge_lengths, &weights, &params);
        let solution = solve_qp(&problem, params.qp.clone()).map_err(MinCurvatureError::Qp)?;
        if solution.len() != alpha.len() || solution.iter().any(|value| !value.is_finite()) {
            return Err(MinCurvatureError::Qp(QpError::NonFiniteData));
        }

        let damping = damping_for_iteration(iteration, &params) as f64;
        let mut candidate_alpha = alpha.clone();
        for (candidate, solved) in candidate_alpha.iter_mut().zip(solution) {
            *candidate += damping * (solved - *candidate);
        }
        clamp_offsets_to_bounds(&mut candidate_alpha, &bounds);

        let candidate_points = apply_offsets(&original_points, &normals, &candidate_alpha);
        let direct = direct_curvatures(&candidate_points, reference.closed)?;
        max_error = linearization
            .rows
            .iter()
            .map(|row| (direct[row.index] - row.predict(&candidate_alpha)).abs())
            .fold(0.0_f64, f64::max);

        if max_error <= params.convergence_tolerance_m_inv as f64 {
            let path = path_from_offsets(
                &original_points,
                &normals,
                &candidate_alpha,
                reference.closed,
            )?;
            let offsets_m = candidate_alpha.iter().map(|value| *value as f32).collect();
            return Ok(MinCurvatureResult {
                path,
                offsets_m,
                iterations: iteration + 1,
                status: MinCurvatureStatus::Converged {
                    max_linearization_error_m_inv: max_error as f32,
                },
            });
        }

        alpha = candidate_alpha;
    }

    let zero_offsets = vec![0.0_f64; original_points.len()];
    let path = path_from_offsets(&original_points, &normals, &zero_offsets, reference.closed)?;
    Ok(MinCurvatureResult {
        path,
        offsets_m: vec![0.0; original_points.len()],
        iterations: params.max_iterations.max(1),
        status: MinCurvatureStatus::ReferenceFallback {
            reason: MinCurvatureFallbackReason::DidNotConverge {
                iterations: params.max_iterations.max(1),
                max_linearization_error_m_inv: max_error as f32,
            },
        },
    })
}

pub(crate) fn squared_curvature_cost(path: &MinCurvaturePath) -> f32 {
    let points: Vec<_> = path.samples.iter().map(|sample| sample.position).collect();
    squared_curvature_cost_points(&points, path.closed).unwrap_or(f32::INFINITY)
}

fn validate_params(params: &MinCurvatureParams) -> Result<(), MinCurvatureError> {
    let damping_values = [
        params.damping_first,
        params.damping_second,
        params.damping_later,
    ];
    if !params.clearance_m.is_finite()
        || params.clearance_m < 0.0
        || !params.max_curvature_m_inv.is_finite()
        || !params.regularization_weight_m_inv4.is_finite()
        || params.regularization_weight_m_inv4 < 0.0
        || !params.max_adjacent_offset_slope.is_finite()
        || params.max_adjacent_offset_slope < 0.0
        || !params.convergence_tolerance_m_inv.is_finite()
        || params.convergence_tolerance_m_inv < 0.0
        || damping_values
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0 || *value > 1.0)
    {
        return Err(MinCurvatureError::InvalidParameters);
    }
    Ok(())
}

fn validate_points(points: &[Point2]) -> Result<(), MinCurvatureError> {
    if points
        .iter()
        .any(|point| !point.x.is_finite() || !point.y.is_finite())
    {
        return Err(MinCurvatureError::NonFiniteInput);
    }
    Ok(())
}

fn alpha_bounds(
    reference: &ReferencePath,
    clearance_m: f32,
) -> Result<Vec<(f64, f64)>, MinCurvatureError> {
    let mut bounds = Vec::with_capacity(reference.samples.len());
    for (index, sample) in reference.samples.iter().enumerate() {
        if !sample.width_left_m.is_finite() || !sample.width_right_m.is_finite() {
            return Err(MinCurvatureError::NonFiniteInput);
        }
        let lower = -sample.width_right_m + clearance_m;
        let upper = sample.width_left_m - clearance_m;
        if lower > upper {
            return Err(MinCurvatureError::InfeasibleBounds {
                index,
                lower_m: lower,
                upper_m: upper,
                clearance_m,
            });
        }
        bounds.push((lower as f64, upper as f64));
    }
    Ok(bounds)
}

fn segment_weights(points: &[Point2], closed: bool) -> Result<Vec<f64>, MinCurvatureError> {
    let n = points.len();
    let mut weights = vec![0.0; n];
    if closed {
        for i in 0..n {
            let prev = (i + n - 1) % n;
            let next = (i + 1) % n;
            let h_prev = points[prev].dist(&points[i]) as f64;
            let h_next = points[i].dist(&points[next]) as f64;
            if h_prev <= MIN_SEGMENT_LENGTH_M || h_next <= MIN_SEGMENT_LENGTH_M {
                return Err(MinCurvatureError::DegenerateGeometry { index: i });
            }
            weights[i] = 0.5 * (h_prev + h_next);
        }
    } else {
        for i in 0..n - 1 {
            let h = points[i].dist(&points[i + 1]) as f64;
            if h <= MIN_SEGMENT_LENGTH_M {
                return Err(MinCurvatureError::DegenerateGeometry { index: i });
            }
            weights[i] += 0.5 * h;
            weights[i + 1] += 0.5 * h;
        }
    }
    Ok(weights)
}

fn adjacent_edge_lengths(
    points: &[Point2],
    closed: bool,
) -> Result<Vec<(usize, usize, f64)>, MinCurvatureError> {
    let mut edges = Vec::with_capacity(if closed {
        points.len()
    } else {
        points.len().saturating_sub(1)
    });
    for i in 0..points.len().saturating_sub(1) {
        let length = points[i].dist(&points[i + 1]) as f64;
        if length <= MIN_SEGMENT_LENGTH_M {
            return Err(MinCurvatureError::DegenerateGeometry { index: i });
        }
        edges.push((i, i + 1, length));
    }
    if closed {
        let last = points.len() - 1;
        let length = points[last].dist(&points[0]) as f64;
        if length <= MIN_SEGMENT_LENGTH_M {
            return Err(MinCurvatureError::DegenerateGeometry { index: last });
        }
        edges.push((last, 0, length));
    }
    Ok(edges)
}

#[derive(Clone, Debug, PartialEq)]
struct CurvatureLinearization {
    rows: Vec<CurvatureRow>,
}

#[derive(Clone, Debug, PartialEq)]
struct CurvatureRow {
    index: usize,
    base: f64,
    coefficients: Vec<(usize, f64)>,
    weight: f64,
}

impl CurvatureRow {
    fn predict(&self, alpha: &[f64]) -> f64 {
        self.base
            + self
                .coefficients
                .iter()
                .map(|(index, coefficient)| coefficient * alpha[*index])
                .sum::<f64>()
    }
}

fn curvature_linearization(
    normals: &[UnitVector2],
    current_alpha: &[f64],
    current_points: &[Point2],
    closed: bool,
    weights: &[f64],
) -> Result<CurvatureLinearization, MinCurvatureError> {
    let n = current_points.len();
    let row_indices: Vec<usize> = if closed {
        (0..n).collect()
    } else {
        (1..n - 1).collect()
    };
    let mut rows = Vec::with_capacity(row_indices.len());
    for i in row_indices {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = if i + 1 == n { 0 } else { i + 1 };
        let h_prev = current_points[prev].dist(&current_points[i]) as f64;
        let h_next = current_points[i].dist(&current_points[next]) as f64;
        if h_prev <= MIN_SEGMENT_LENGTH_M || h_next <= MIN_SEGMENT_LENGTH_M {
            return Err(MinCurvatureError::DegenerateGeometry { index: i });
        }
        let first = first_derivative_coefficients(h_prev, h_next);
        let second = second_derivative_coefficients(h_prev, h_next);
        let x1 = first[0] * current_points[prev].x as f64
            + first[1] * current_points[i].x as f64
            + first[2] * current_points[next].x as f64;
        let y1 = first[0] * current_points[prev].y as f64
            + first[1] * current_points[i].y as f64
            + first[2] * current_points[next].y as f64;
        let x2 = second[0] * current_points[prev].x as f64
            + second[1] * current_points[i].x as f64
            + second[2] * current_points[next].x as f64;
        let y2 = second[0] * current_points[prev].y as f64
            + second[1] * current_points[i].y as f64
            + second[2] * current_points[next].y as f64;
        let speed_sq = x1 * x1 + y1 * y1;
        if speed_sq <= 1.0e-12 {
            return Err(MinCurvatureError::DegenerateGeometry { index: i });
        }
        let denom = speed_sq.powf(1.5);
        let numerator = x1 * y2 - y1 * x2;
        let curvature = numerator / denom;
        let indices = [prev, i, next];
        let mut coefficients = Vec::with_capacity(3);
        for ((&index, &first_coeff), &second_coeff) in
            indices.iter().zip(first.iter()).zip(second.iter())
        {
            let normal = normals[index];
            let dx1 = first_coeff * normal.x as f64;
            let dy1 = first_coeff * normal.y as f64;
            let dx2 = second_coeff * normal.x as f64;
            let dy2 = second_coeff * normal.y as f64;
            let d_numerator = dx1 * y2 + x1 * dy2 - dy1 * x2 - y1 * dx2;
            let d_speed_sq = 2.0 * (x1 * dx1 + y1 * dy1);
            let d_curvature = d_numerator / denom - curvature * 1.5 * d_speed_sq / speed_sq;
            coefficients.push((index, d_curvature));
        }
        let base = curvature
            - coefficients
                .iter()
                .map(|(index, coefficient)| coefficient * current_alpha[*index])
                .sum::<f64>();
        rows.push(CurvatureRow {
            index: i,
            base,
            coefficients,
            weight: weights[i],
        });
    }
    Ok(CurvatureLinearization { rows })
}

fn build_problem(
    linearization: &CurvatureLinearization,
    bounds: &[(f64, f64)],
    edge_lengths: &[(usize, usize, f64)],
    weights: &[f64],
    params: &MinCurvatureParams,
) -> QuadraticProgram {
    let n = bounds.len();
    let mut hessian = BTreeMap::<(usize, usize), f64>::new();
    let mut linear = vec![0.0; n];
    for row in &linearization.rows {
        for &(j, cj) in &row.coefficients {
            linear[j] += 2.0 * row.weight * row.base * cj;
            for &(k, ck) in &row.coefficients {
                *hessian.entry((j, k)).or_insert(0.0) += row.weight * cj * ck;
            }
        }
    }
    for (i, weight) in weights.iter().enumerate() {
        *hessian.entry((i, i)).or_insert(0.0) +=
            params.regularization_weight_m_inv4 as f64 * weight;
    }

    let curvature_constraints = if params.max_curvature_m_inv > 0.0 {
        linearization.rows.len() * 2
    } else {
        0
    };
    let mut inequalities =
        Vec::with_capacity(n * 2 + edge_lengths.len() * 2 + curvature_constraints);
    for (i, &(lower, upper)) in bounds.iter().enumerate() {
        inequalities.push(LinearInequality {
            coefficients: vec![(i, 1.0)],
            upper_bound: upper,
        });
        inequalities.push(LinearInequality {
            coefficients: vec![(i, -1.0)],
            upper_bound: -lower,
        });
    }

    if params.max_adjacent_offset_slope > 0.0 {
        let slope = params.max_adjacent_offset_slope as f64;
        for &(i, j, length) in edge_lengths {
            let limit = slope * length;
            inequalities.push(LinearInequality {
                coefficients: vec![(j, 1.0), (i, -1.0)],
                upper_bound: limit,
            });
            inequalities.push(LinearInequality {
                coefficients: vec![(i, 1.0), (j, -1.0)],
                upper_bound: limit,
            });
        }
    }

    if params.max_curvature_m_inv > 0.0 {
        let limit = params.max_curvature_m_inv as f64;
        for row in &linearization.rows {
            inequalities.push(LinearInequality {
                coefficients: row.coefficients.clone(),
                upper_bound: limit - row.base,
            });
            inequalities.push(LinearInequality {
                coefficients: row
                    .coefficients
                    .iter()
                    .map(|(index, coefficient)| (*index, -*coefficient))
                    .collect(),
                upper_bound: limit + row.base,
            });
        }
    }

    QuadraticProgram {
        variables: n,
        hessian_entries: hessian
            .into_iter()
            .map(|((row, col), value)| (row, col, 2.0 * value))
            .collect(),
        linear,
        inequalities,
    }
}

fn damping_for_iteration(iteration: usize, params: &MinCurvatureParams) -> f32 {
    match iteration {
        0 => params.damping_first,
        1 => params.damping_second,
        _ => params.damping_later,
    }
}

fn clamp_offsets_to_bounds(alpha: &mut [f64], bounds: &[(f64, f64)]) {
    for (value, &(lower, upper)) in alpha.iter_mut().zip(bounds) {
        *value = value.clamp(lower, upper);
    }
}

fn apply_offsets(points: &[Point2], normals: &[UnitVector2], alpha: &[f64]) -> Vec<Point2> {
    points
        .iter()
        .zip(normals)
        .zip(alpha)
        .map(|((point, normal), alpha)| {
            Point2::new(
                point.x + (*alpha as f32) * normal.x,
                point.y + (*alpha as f32) * normal.y,
            )
        })
        .collect()
}

fn path_from_offsets(
    points: &[Point2],
    normals: &[UnitVector2],
    alpha: &[f64],
    closed: bool,
) -> Result<MinCurvaturePath, MinCurvatureError> {
    let shifted = apply_offsets(points, normals, alpha);
    let curvatures = direct_curvatures(&shifted, closed)?;
    let mut s_values = vec![0.0; shifted.len()];
    let mut length_m = 0.0;
    for i in 1..shifted.len() {
        length_m += shifted[i - 1].dist(&shifted[i]);
        s_values[i] = length_m;
    }
    if closed {
        length_m += shifted[shifted.len() - 1].dist(&shifted[0]);
    }

    let mut samples = Vec::with_capacity(shifted.len());
    for i in 0..shifted.len() {
        let tangent = tangent_at(&shifted, i, closed)?;
        samples.push(MinCurvatureSample {
            position: shifted[i],
            tangent,
            normal: tangent.left_normal(),
            curvature_m_inv: curvatures[i] as f32,
            alpha_m: alpha[i] as f32,
            s_m: s_values[i],
        });
    }

    Ok(MinCurvaturePath {
        samples,
        closed,
        length_m,
    })
}

fn tangent_at(
    points: &[Point2],
    index: usize,
    closed: bool,
) -> Result<UnitVector2, MinCurvatureError> {
    let n = points.len();
    let (a, b) = if closed {
        (points[(index + n - 1) % n], points[(index + 1) % n])
    } else if index == 0 {
        (points[index], points[index + 1])
    } else if index + 1 == n {
        (points[index - 1], points[index])
    } else {
        (points[index - 1], points[index + 1])
    };
    UnitVector2::new(b.x - a.x, b.y - a.y).ok_or(MinCurvatureError::DegenerateGeometry { index })
}

fn direct_curvatures(points: &[Point2], closed: bool) -> Result<Vec<f64>, MinCurvatureError> {
    let n = points.len();
    let mut curvatures = vec![0.0; n];
    let row_indices: Vec<usize> = if closed {
        (0..n).collect()
    } else {
        (1..n - 1).collect()
    };
    for i in row_indices {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = if i + 1 == n { 0 } else { i + 1 };
        let h_prev = points[prev].dist(&points[i]) as f64;
        let h_next = points[i].dist(&points[next]) as f64;
        if h_prev <= MIN_SEGMENT_LENGTH_M || h_next <= MIN_SEGMENT_LENGTH_M {
            return Err(MinCurvatureError::DegenerateGeometry { index: i });
        }
        let first = first_derivative_coefficients(h_prev, h_next);
        let second = second_derivative_coefficients(h_prev, h_next);
        let dx = first[0] * points[prev].x as f64
            + first[1] * points[i].x as f64
            + first[2] * points[next].x as f64;
        let dy = first[0] * points[prev].y as f64
            + first[1] * points[i].y as f64
            + first[2] * points[next].y as f64;
        let ddx = second[0] * points[prev].x as f64
            + second[1] * points[i].x as f64
            + second[2] * points[next].x as f64;
        let ddy = second[0] * points[prev].y as f64
            + second[1] * points[i].y as f64
            + second[2] * points[next].y as f64;
        let speed_sq = dx * dx + dy * dy;
        if speed_sq <= 1.0e-12 {
            return Err(MinCurvatureError::DegenerateGeometry { index: i });
        }
        curvatures[i] = (dx * ddy - dy * ddx) / speed_sq.powf(1.5);
    }
    Ok(curvatures)
}

fn squared_curvature_cost_points(
    points: &[Point2],
    closed: bool,
) -> Result<f32, MinCurvatureError> {
    let curvatures = direct_curvatures(points, closed)?;
    let weights = segment_weights(points, closed)?;
    Ok(curvatures
        .into_iter()
        .zip(weights)
        .map(|(curvature, weight)| (curvature * curvature * weight) as f32)
        .sum())
}

fn first_derivative_coefficients(h_prev: f64, h_next: f64) -> [f64; 3] {
    let total = h_prev + h_next;
    [
        -h_next / (h_prev * total),
        (h_next - h_prev) / (h_prev * h_next),
        h_prev / (h_next * total),
    ]
}

fn second_derivative_coefficients(h_prev: f64, h_next: f64) -> [f64; 3] {
    let total = h_prev + h_next;
    [
        2.0 / (h_prev * total),
        -2.0 / (h_prev * h_next),
        2.0 / (h_next * total),
    ]
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;
    use crate::raceline::reference::ReferenceSample;

    fn circular_reference(radius_m: f32, width_left_m: f32, width_right_m: f32) -> ReferencePath {
        let n = 96;
        let mut samples = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * PI * i as f32 / n as f32;
            let position = Point2::new(radius_m * theta.cos(), radius_m * theta.sin());
            let tangent = UnitVector2::new(-theta.sin(), theta.cos()).unwrap();
            samples.push(ReferenceSample {
                position,
                tangent,
                normal: tangent.left_normal(),
                width_left_m,
                width_right_m,
                s_m: radius_m * theta,
            });
        }
        ReferencePath::new(samples, true, 2.0 * PI * radius_m, Vec::new())
    }

    fn open_straight_reference() -> ReferencePath {
        let mut samples = Vec::new();
        for i in 0..16 {
            let position = Point2::new(i as f32, 0.0);
            samples.push(ReferenceSample {
                position,
                tangent: UnitVector2::new(1.0, 0.0).unwrap(),
                normal: UnitVector2::new(0.0, 1.0).unwrap(),
                width_left_m: 1.0,
                width_right_m: 1.0,
                s_m: i as f32,
            });
        }
        ReferencePath::new(samples, false, 15.0, Vec::new())
    }

    fn chicane_reference() -> ReferencePath {
        let n = 120;
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * PI * i as f32 / n as f32;
            let radius = 4.0 + 0.45 * (3.0 * theta).sin();
            points.push(Point2::new(radius * theta.cos(), radius * theta.sin()));
        }
        reference_from_closed_points(points, 0.8, 0.8)
    }

    fn reference_from_closed_points(
        points: Vec<Point2>,
        width_left_m: f32,
        width_right_m: f32,
    ) -> ReferencePath {
        let mut samples = Vec::with_capacity(points.len());
        let mut s_m = 0.0;
        for i in 0..points.len() {
            if i > 0 {
                s_m += points[i - 1].dist(&points[i]);
            }
            let prev = points[(i + points.len() - 1) % points.len()];
            let next = points[(i + 1) % points.len()];
            let tangent = UnitVector2::new(next.x - prev.x, next.y - prev.y).unwrap();
            samples.push(ReferenceSample {
                position: points[i],
                tangent,
                normal: tangent.left_normal(),
                width_left_m,
                width_right_m,
                s_m,
            });
        }
        let length_m = s_m + points[points.len() - 1].dist(&points[0]);
        ReferencePath::new(samples, true, length_m, Vec::new())
    }

    #[test]
    fn circular_annulus_offsets_toward_larger_radius() {
        let reference = circular_reference(3.0, 1.0, 1.4);
        let result = optimize_min_curvature(
            &reference,
            MinCurvatureParams {
                clearance_m: 0.1,
                convergence_tolerance_m_inv: 0.03,
                ..MinCurvatureParams::default()
            },
        )
        .unwrap();

        assert!(matches!(
            result.status,
            MinCurvatureStatus::Converged { .. }
        ));
        let mean_alpha = result.offsets_m.iter().sum::<f32>() / result.offsets_m.len() as f32;
        let mean_radius = result
            .path
            .samples
            .iter()
            .map(|sample| sample.position.x.hypot(sample.position.y))
            .sum::<f32>()
            / result.path.samples.len() as f32;
        let original = path_from_offsets(
            &reference
                .samples
                .iter()
                .map(|sample| sample.position)
                .collect::<Vec<_>>(),
            &reference
                .samples
                .iter()
                .map(|sample| sample.normal)
                .collect::<Vec<_>>(),
            &vec![0.0; reference.samples.len()],
            true,
        )
        .unwrap();

        assert!(mean_alpha < -0.05, "mean_alpha={mean_alpha}");
        assert!(mean_radius > 3.05, "mean_radius={mean_radius}");
        assert!(squared_curvature_cost(&result.path) < squared_curvature_cost(&original));
    }

    #[test]
    fn straight_corridor_stays_centered_and_finite() {
        let reference = open_straight_reference();
        let result = optimize_min_curvature(
            &reference,
            MinCurvatureParams {
                clearance_m: 0.1,
                ..MinCurvatureParams::default()
            },
        )
        .unwrap();

        assert!(matches!(
            result.status,
            MinCurvatureStatus::Converged { .. }
        ));
        for sample in &result.path.samples {
            assert!(sample.position.x.is_finite());
            assert!(sample.position.y.abs() < 1.0e-4, "{sample:?}");
            assert!(sample.curvature_m_inv.abs() < 1.0e-4, "{sample:?}");
        }
    }

    #[test]
    fn chicane_reduces_curvature_cost() {
        let reference = chicane_reference();
        let original_points: Vec<_> = reference
            .samples
            .iter()
            .map(|sample| sample.position)
            .collect();
        let original_cost = squared_curvature_cost_points(&original_points, true).unwrap();
        let result = optimize_min_curvature(
            &reference,
            MinCurvatureParams {
                clearance_m: 0.1,
                convergence_tolerance_m_inv: 0.03,
                ..MinCurvatureParams::default()
            },
        )
        .unwrap();

        assert!(matches!(
            result.status,
            MinCurvatureStatus::Converged { .. }
        ));
        let optimized_cost = squared_curvature_cost(&result.path);
        assert!(
            optimized_cost < original_cost,
            "original={original_cost} optimized={optimized_cost}"
        );
    }

    #[test]
    fn infeasible_bounds_return_clear_error() {
        let reference = circular_reference(3.0, 0.03, 0.03);
        let err = optimize_min_curvature(
            &reference,
            MinCurvatureParams {
                clearance_m: 0.1,
                ..MinCurvatureParams::default()
            },
        )
        .unwrap_err();

        assert!(matches!(err, MinCurvatureError::InfeasibleBounds { .. }));
    }

    #[test]
    fn infeasible_curvature_bound_is_reported() {
        let reference = circular_reference(3.0, 0.1, 0.1);
        let err = optimize_min_curvature(
            &reference,
            MinCurvatureParams {
                clearance_m: 0.05,
                max_curvature_m_inv: 0.01,
                ..MinCurvatureParams::default()
            },
        )
        .unwrap_err();

        assert!(matches!(err, MinCurvatureError::Qp(_)));
    }
}
