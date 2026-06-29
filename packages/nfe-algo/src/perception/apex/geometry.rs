use nfe_core::sensors::LidarPoint;
use nfe_core::wrap_angle;

use super::scan::{ApexWall, HermiteBounds};

#[derive(Clone, Copy, Debug)]
pub(super) struct OppositeParams {
    pub(super) max_dist_error_m: f32,
    pub(super) prefer_nearer: bool,
    pub(super) wall_clearance_m: f32,
}

pub(super) struct ApexGeometry;

impl ApexGeometry {
    pub(super) fn opposite_point(
        wall: ApexWall<'_>,
        breakpoint: &LidarPoint,
        timestamp_us: u64,
        params: OppositeParams,
    ) -> Option<LidarPoint> {
        let raw = if let Some(bounds) = wall.bounding_points(breakpoint) {
            opposite_from_bounds(bounds, breakpoint, timestamp_us, params)
        } else {
            scan_based_opposite_fallback(wall, timestamp_us)?
        };

        Some(pull_toward_origin(raw, params.wall_clearance_m))
    }

    pub(super) fn polar_midpoint(
        apex: LidarPoint,
        opposite: LidarPoint,
        timestamp_us: u64,
    ) -> LidarPoint {
        let target_dist = (apex.dist_m + opposite.dist_m) / 2.0;
        let angle_diff = opposite.angle_diff(&apex);
        let target_angle = wrap_angle(apex.angle_rad + angle_diff / 2.0);

        LidarPoint::from_polar(target_dist, target_angle, timestamp_us)
    }

    pub(super) fn cartesian_midpoint(
        apex: LidarPoint,
        opposite: LidarPoint,
        timestamp_us: u64,
    ) -> LidarPoint {
        let x = (apex.x + opposite.x) / 2.0;
        let y = (apex.y + opposite.y) / 2.0;
        LidarPoint {
            x,
            y,
            dist_m: x.hypot(y),
            angle_rad: y.atan2(x),
            timestamp_us,
        }
    }
}

fn opposite_from_bounds(
    bounds: HermiteBounds<'_>,
    breakpoint: &LidarPoint,
    timestamp_us: u64,
    params: OppositeParams,
) -> LidarPoint {
    if (bounds.a.dist_m - bounds.b.dist_m).abs() >= params.max_dist_error_m {
        *select_opposite_bound(bounds, breakpoint, params.prefer_nearer)
    } else {
        let angle = hermite_angle_at_range(
            bounds.a,
            bounds.b,
            bounds.prev,
            bounds.next,
            breakpoint.dist_m,
        );
        LidarPoint::from_polar(breakpoint.dist_m, angle, timestamp_us)
    }
}

fn select_opposite_bound<'a>(
    bounds: HermiteBounds<'a>,
    breakpoint: &LidarPoint,
    prefer_nearer: bool,
) -> &'a LidarPoint {
    if prefer_nearer {
        let a_err = (bounds.a.dist_m - breakpoint.dist_m).abs();
        let b_err = (bounds.b.dist_m - breakpoint.dist_m).abs();
        if a_err <= b_err {
            bounds.a
        } else {
            bounds.b
        }
    } else if bounds.a.dist_m > bounds.b.dist_m {
        bounds.a
    } else {
        bounds.b
    }
}

fn hermite_angle_at_range(
    a: &LidarPoint,
    b: &LidarPoint,
    prev: &LidarPoint,
    next: &LidarPoint,
    range_m: f32,
) -> f32 {
    let d_range = b.dist_m - a.dist_m;
    if d_range.abs() < f32::EPSILON {
        return a.angle_rad;
    }

    let prev_range = a.dist_m - prev.dist_m;
    let next_range = next.dist_m - b.dist_m;
    let dot_theta_a = if prev_range.abs() < f32::EPSILON {
        0.0
    } else {
        a.angle_diff(prev) / prev_range
    };
    let dot_theta_b = if next_range.abs() < f32::EPSILON {
        0.0
    } else {
        next.angle_diff(b) / next_range
    };

    let t = ((range_m - a.dist_m) / d_range).clamp(0.0, 1.0);
    let t2 = t * t;
    let t3 = t2 * t;

    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = -2.0 * t3 + 3.0 * t2;
    let h01 = (t3 - 2.0 * t2 + t) * d_range;
    let h11 = (t3 - t2) * d_range;

    wrap_angle(
        h00 * a.angle_rad
            + h10 * (a.angle_rad + b.angle_diff(a))
            + h01 * dot_theta_a
            + h11 * dot_theta_b,
    )
}

fn scan_based_opposite_fallback(wall: ApexWall<'_>, timestamp_us: u64) -> Option<LidarPoint> {
    if wall.points().is_empty() {
        return None;
    }

    let dist_m = median_range_m(wall.points());
    let angle_rad = angular_centroid_rad(wall.points());
    Some(LidarPoint::from_polar(dist_m, angle_rad, timestamp_us))
}

fn median_range_m(points: &[LidarPoint]) -> f32 {
    let mut ranges: Vec<_> = points.iter().map(|p| p.dist_m).collect();
    ranges.sort_by(f32::total_cmp);
    let mid = ranges.len() / 2;
    if ranges.len() % 2 == 0 {
        (ranges[mid - 1] + ranges[mid]) / 2.0
    } else {
        ranges[mid]
    }
}

fn angular_centroid_rad(points: &[LidarPoint]) -> f32 {
    let (sin_sum, cos_sum) = points.iter().fold((0.0_f32, 0.0_f32), |(sin, cos), p| {
        (sin + p.angle_rad.sin(), cos + p.angle_rad.cos())
    });

    if sin_sum.hypot(cos_sum) <= f32::EPSILON {
        wrap_angle(points.iter().map(|p| p.angle_rad).sum::<f32>() / points.len() as f32)
    } else {
        sin_sum.atan2(cos_sum)
    }
}

fn pull_toward_origin(point: LidarPoint, margin_m: f32) -> LidarPoint {
    point.with_distance((point.dist_m - margin_m.max(0.0)).max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn point(angle_rad: f32, dist_m: f32) -> LidarPoint {
        LidarPoint::from_polar(dist_m, angle_rad, 0)
    }

    #[test]
    fn polar_midpoint_between_sixty_degree_bounds_points_forward() {
        let apex = point(PI / 3.0, 2.0);
        let opposite = point(-PI / 3.0, 2.0);

        let target = ApexGeometry::polar_midpoint(apex, opposite, 7);

        assert!(target.angle_rad.abs() < 1e-6, "target={target:?}");
        assert!(target.y.abs() < 1e-6, "target={target:?}");
        assert!((target.x - 2.0).abs() < 1e-6, "target={target:?}");
    }

    #[test]
    fn opposite_bound_selection_can_prefer_breakpoint_distance() {
        let prev = point(-0.4, 2.0);
        let a = point(-0.2, 4.0);
        let b = point(0.2, 1.2);
        let next = point(0.4, 1.0);
        let breakpoint = point(0.0, 1.0);
        let bounds = HermiteBounds {
            prev: &prev,
            a: &a,
            b: &b,
            next: &next,
        };

        let selected = select_opposite_bound(bounds, &breakpoint, true);
        assert_eq!(selected.angle_rad, b.angle_rad);

        let selected = select_opposite_bound(bounds, &breakpoint, false);
        assert_eq!(selected.angle_rad, a.angle_rad);
    }

    #[test]
    fn wall_clearance_pulls_opposite_toward_origin() {
        let original = point(0.25, 1.0);

        let pulled = pull_toward_origin(original, 0.15);

        assert!((pulled.dist_m - 0.85).abs() < 1e-6, "pulled={pulled:?}");
        assert!((pulled.angle_rad - original.angle_rad).abs() < 1e-6);
        assert!((pulled.x - 0.85 * original.angle_rad.cos()).abs() < 1e-6);
        assert!((pulled.y - 0.85 * original.angle_rad.sin()).abs() < 1e-6);
    }

    #[test]
    fn scan_fallback_uses_wall_median_range_and_angular_centroid() {
        let points = [point(0.0, 1.0), point(0.2, 5.0), point(0.4, 3.0)];
        let wall = ApexWall { points: &points };

        let fallback = scan_based_opposite_fallback(wall, 42).expect("fallback point");

        assert!(
            (fallback.dist_m - 3.0).abs() < 1e-6,
            "fallback={fallback:?}"
        );
        assert!(
            (fallback.angle_rad - 0.2).abs() < 1e-6,
            "fallback={fallback:?}"
        );
        assert_eq!(fallback.timestamp_us, 42);
    }
}
