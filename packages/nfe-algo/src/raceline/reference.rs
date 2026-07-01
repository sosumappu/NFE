#![allow(dead_code)]

use nfe_core::mapping::DistanceField;
use nfe_core::Point2;

use crate::raceline::contour::ClosedContour;
use crate::raceline::geometry::UnitVector2;
use crate::raceline::grid::BinaryMask;
use crate::raceline::savgol::{smooth_circular, SavitzkyGolayError, SavitzkyGolayParams};

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ReferenceSample {
    pub position: Point2,
    pub tangent: UnitVector2,
    /// Left-hand normal of `tangent`. Positive lateral offsets follow this
    /// repo-local convention because the rest of the control stack treats left
    /// as positive; the paper only requires a consistent normal direction.
    pub normal: UnitVector2,
    pub width_left_m: f32,
    pub width_right_m: f32,
    pub s_m: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum WidthSide {
    Left,
    Right,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ReferencePathDiagnostic {
    WidthRaySaturated {
        index: usize,
        side: WidthSide,
        point: Point2,
        max_width_m: f32,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ReferencePath {
    pub samples: Vec<ReferenceSample>,
    /// Production extraction is expected to return closed loops. The flag keeps
    /// straight/open synthetic references representable in unit tests.
    pub closed: bool,
    pub length_m: f32,
    pub diagnostics: Vec<ReferencePathDiagnostic>,
}

impl ReferencePath {
    pub(crate) fn new(
        samples: Vec<ReferenceSample>,
        closed: bool,
        length_m: f32,
        diagnostics: Vec<ReferencePathDiagnostic>,
    ) -> Self {
        Self {
            samples,
            closed,
            length_m,
            diagnostics,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ReferencePathParams {
    pub smoothing: SavitzkyGolayParams,
    /// Non-positive means half the occupancy-grid resolution.
    pub width_step_m: f32,
    pub max_width_m: f32,
}

impl Default for ReferencePathParams {
    fn default() -> Self {
        Self {
            smoothing: SavitzkyGolayParams::default(),
            width_step_m: 0.0,
            max_width_m: 5.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ReferencePathError {
    NotImplemented,
    EmptyContour,
    Smoothing(SavitzkyGolayError),
    DegenerateTangent,
    PointOutsideFreeSpace { index: usize, point: Point2 },
}

pub(crate) fn reference_path_from_contour(
    mask: &BinaryMask,
    contour: &ClosedContour,
    params: ReferencePathParams,
) -> Result<ReferencePath, ReferencePathError> {
    if contour.points.len() < 3 {
        return Err(ReferencePathError::EmptyContour);
    }

    let xs: Vec<_> = contour.points.iter().map(|p| p.x).collect();
    let ys: Vec<_> = contour.points.iter().map(|p| p.y).collect();
    let xs = smooth_circular(&xs, params.smoothing).map_err(ReferencePathError::Smoothing)?;
    let ys = smooth_circular(&ys, params.smoothing).map_err(ReferencePathError::Smoothing)?;
    let points: Vec<_> = xs
        .into_iter()
        .zip(ys)
        .map(|(x, y)| Point2::new(x, y))
        .collect();

    let mut s_values = Vec::with_capacity(points.len());
    let mut s = 0.0;
    for i in 0..points.len() {
        if i > 0 {
            s += points[i - 1].dist(&points[i]);
        }
        s_values.push(s);
    }
    let length_m = s + points[points.len() - 1].dist(&points[0]);

    let step_m = if params.width_step_m > 0.0 {
        params.width_step_m
    } else {
        0.5 * mask.spec.resolution_m
    };
    let mut samples = Vec::with_capacity(points.len());
    let mut diagnostics = Vec::new();
    for i in 0..points.len() {
        let point = points[i];
        if !mask.is_free_world(point) {
            return Err(ReferencePathError::PointOutsideFreeSpace { index: i, point });
        }
        let prev = points[(i + points.len() - 1) % points.len()];
        let next = points[(i + 1) % points.len()];
        let tangent = UnitVector2::new(next.x - prev.x, next.y - prev.y)
            .ok_or(ReferencePathError::DegenerateTangent)?;
        let normal = tangent.left_normal();
        let left = ray_width(mask, point, normal, step_m, params.max_width_m);
        let right = ray_width(
            mask,
            point,
            UnitVector2 {
                x: -normal.x,
                y: -normal.y,
            },
            step_m,
            params.max_width_m,
        );
        if left.saturated {
            diagnostics.push(ReferencePathDiagnostic::WidthRaySaturated {
                index: i,
                side: WidthSide::Left,
                point,
                max_width_m: params.max_width_m.max(0.0),
            });
        }
        if right.saturated {
            diagnostics.push(ReferencePathDiagnostic::WidthRaySaturated {
                index: i,
                side: WidthSide::Right,
                point,
                max_width_m: params.max_width_m.max(0.0),
            });
        }
        samples.push(ReferenceSample {
            position: point,
            tangent,
            normal,
            width_left_m: left.width_m,
            width_right_m: right.width_m,
            s_m: s_values[i],
        });
    }

    Ok(ReferencePath::new(samples, true, length_m, diagnostics))
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct RayWidth {
    width_m: f32,
    saturated: bool,
}

fn ray_width(
    mask: &BinaryMask,
    origin: Point2,
    direction: UnitVector2,
    step_m: f32,
    max_width_m: f32,
) -> RayWidth {
    let step_m = step_m.max(mask.spec.resolution_m * 0.1).max(1.0e-4);
    let max_width_m = max_width_m.max(0.0);
    let mut last_free = 0.0;
    let mut d = step_m;
    while d <= max_width_m {
        let point = Point2::new(origin.x + direction.x * d, origin.y + direction.y * d);
        if !mask.is_free_world(point) {
            return RayWidth {
                width_m: last_free,
                saturated: false,
            };
        }
        last_free = d;
        d += step_m;
    }
    RayWidth {
        width_m: max_width_m,
        saturated: true,
    }
}

pub(crate) fn extract_reference_path(
    _mask: &BinaryMask,
    _inner_distance: &DistanceField,
    _outer_distance: &DistanceField,
) -> Result<ReferencePath, ReferencePathError> {
    Err(ReferencePathError::NotImplemented)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raceline::grid::GridSpec;

    fn corridor_mask() -> BinaryMask {
        let spec = GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width: 9,
            height: 7,
        };
        let mut free = vec![false; spec.len()];
        for y in 1..6 {
            for x in 1..8 {
                free[spec.index(x, y)] = true;
            }
        }
        BinaryMask::new(spec, free)
    }

    #[test]
    fn contour_to_reference_computes_tangents_normals_and_widths() {
        let mask = corridor_mask();
        let contour = ClosedContour {
            points: vec![
                Point2::new(2.5, 3.5),
                Point2::new(3.5, 3.5),
                Point2::new(4.5, 3.5),
                Point2::new(5.5, 3.5),
                Point2::new(6.5, 3.5),
            ],
            length_m: 8.0,
        };

        let reference = reference_path_from_contour(
            &mask,
            &contour,
            ReferencePathParams {
                smoothing: SavitzkyGolayParams {
                    window_points: 3,
                    polynomial_order: 1,
                },
                width_step_m: 0.5,
                max_width_m: 5.0,
            },
        )
        .unwrap();

        assert_eq!(reference.samples.len(), 5);
        let sample = &reference.samples[2];
        assert!(sample.tangent.x.abs() > 0.9);
        assert!(sample.normal.y.abs() > 0.9);
        assert!((sample.width_left_m - 1.5).abs() <= 0.5, "{sample:?}");
        assert!((sample.width_right_m - 2.5).abs() <= 0.5, "{sample:?}");
        assert!(reference.length_m.is_finite());
        assert!(reference.diagnostics.is_empty());
    }

    #[test]
    fn width_saturation_is_reported_explicitly() {
        let spec = GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width: 20,
            height: 20,
        };
        let mask = BinaryMask::new(spec, vec![true; spec.len()]);
        let contour = ClosedContour {
            points: vec![
                Point2::new(9.5, 9.5),
                Point2::new(10.5, 9.5),
                Point2::new(11.5, 10.5),
                Point2::new(11.5, 11.5),
                Point2::new(10.5, 12.5),
                Point2::new(9.5, 12.5),
                Point2::new(8.5, 11.5),
                Point2::new(8.5, 10.5),
            ],
            length_m: 8.0,
        };

        let reference = reference_path_from_contour(
            &mask,
            &contour,
            ReferencePathParams {
                smoothing: SavitzkyGolayParams {
                    window_points: 3,
                    polynomial_order: 1,
                },
                width_step_m: 0.25,
                max_width_m: 1.0,
            },
        )
        .unwrap();

        assert_eq!(reference.samples.len(), 8);
        assert_eq!(reference.diagnostics.len(), 16);
        assert!(reference.diagnostics.iter().any(|diagnostic| matches!(
            diagnostic,
            ReferencePathDiagnostic::WidthRaySaturated {
                side: WidthSide::Left,
                max_width_m,
                ..
            } if (*max_width_m - 1.0).abs() < 1.0e-6
        )));
        assert!(reference.diagnostics.iter().any(|diagnostic| matches!(
            diagnostic,
            ReferencePathDiagnostic::WidthRaySaturated {
                side: WidthSide::Right,
                max_width_m,
                ..
            } if (*max_width_m - 1.0).abs() < 1.0e-6
        )));
    }

    #[test]
    fn reference_rejects_smoothed_points_outside_free_space() {
        let mask = corridor_mask();
        let contour = ClosedContour {
            points: vec![
                Point2::new(0.1, 0.1),
                Point2::new(0.2, 0.1),
                Point2::new(0.3, 0.1),
            ],
            length_m: 0.4,
        };

        let err = reference_path_from_contour(
            &mask,
            &contour,
            ReferencePathParams {
                smoothing: SavitzkyGolayParams {
                    window_points: 3,
                    polynomial_order: 1,
                },
                ..ReferencePathParams::default()
            },
        )
        .unwrap_err();

        assert!(matches!(
            err,
            ReferencePathError::PointOutsideFreeSpace { .. }
        ));
    }
}
