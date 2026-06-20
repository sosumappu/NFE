use crate::types::LidarCloudView;

#[derive(Default)]

pub struct Mpt {}

impl Mpt {
    pub fn new() -> Self {
        Self {}
    }

    // Returns the heading error and the distance of the heading error point
    pub fn compute(&self, cloud: &LidarCloudView) -> (f32, f32) {
        let len = cloud.points.len();
        let breakpoint_opt = cloud.find_breakpoint();
        if breakpoint_opt.is_none() || len < 4 {
            return (0.0, 0.0);
        }

        let breakpoint = breakpoint_opt.unwrap();
        let theta_opp = breakpoint.opposite();

        // modulo to have the circular continuity
        let b_idx = cloud.points.partition_point(|b| b.angle_rad <= theta_opp) % len;
        let a_idx = (b_idx + len - 1) % len;

        let prev_idx = (a_idx + len - 1) % len;
        let next_idx = (b_idx + 1) % len;

        let p_prev = &cloud.points[prev_idx];
        let p_a = &cloud.points[a_idx];
        let p_b = &cloud.points[b_idx];
        let p_next = &cloud.points[next_idx];

        // interpolate distance
        let dist_p_opp = p_a.hermit_interpolation(p_b, p_prev, p_next, theta_opp);

        // get cartesian coords
        let x_opp = dist_p_opp * theta_opp.cos();
        let y_opp = dist_p_opp * theta_opp.sin();
        // avg
        let x_g = (x_opp + breakpoint.x) / 2.0;
        let y_g = (y_opp + breakpoint.y) / 2.0;

        (y_g.hypot(x_g), y_g.atan2(x_g))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{LidarCloudView, LidarPoint};
    use proptest::prelude::*;

    fn make_point(angle_rad: f32, dist_m: f32) -> LidarPoint {
        LidarPoint {
            x: dist_m * angle_rad.cos(),
            y: -dist_m * angle_rad.sin(),
            dist_m,
            angle_rad,
            timestamp_us: 0,
        }
    }

    #[test]
    fn straight_corridor_gives_zero_heading_error() {
        // Symmetric walls left/right → car centered → heading_err ≈ 0
        let points = vec![
            make_point((-90.0f32).to_radians(), 0.5),
            make_point((-45.0f32).to_radians(), 0.7),
            make_point((0.0f32).to_radians(), 2.0),
            make_point((45.0f32).to_radians(), 0.7),
            make_point((90.0f32).to_radians(), 0.5),
        ];
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let mpt = Mpt::new();
        let (_, heading_err) = mpt.compute(&cloud);
        assert!(heading_err.abs() < 0.05, "heading_err={heading_err}");
    }

    #[test]
    fn too_few_points_returns_zero() {
        let points = vec![make_point(0.0, 1.0), make_point(10.0f32.to_radians(), 1.0)];
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let mpt = Mpt::new();
        assert_eq!(mpt.compute(&cloud), (0.0, 0.0));
    }

    #[test]
    fn no_breakpoint_returns_zero() {
        // Perfectly uniform circle — no derivative spike anywhere
        let points: Vec<_> = (0..36).map(|i| make_point((i as f32 * 10.0).to_radians(), 1.0)).collect();
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let mpt = Mpt::new();
        let (dist, _) = mpt.compute(&cloud);
        assert_eq!(dist, 0.0);
    }

    proptest! {
        #[test]
        fn compute_returns_nonneg_and_angle_bounds(
            // generate sorted angles in (-PI, PI]
            angs in prop::collection::vec(-3.0f32..3.0f32, 4..20),
            dists in prop::collection::vec(0.01f32..50.0f32, 4..20)
        ) {
            use std::f32::consts::PI;
            // pair up to create points with increasing angles
            let mut pairs: Vec<(f32,f32)> = angs.into_iter().zip(dists.into_iter()).collect();
            // sort by angle
            pairs.sort_by(|a,b| a.0.partial_cmp(&b.0).unwrap());
            let points: Vec<LidarPoint> = pairs.iter().map(|(a,d)| make_point(*a, *d)).collect();
            let cloud = LidarCloudView { points: &points, timestamp_us: 0 };
            let mpt = Mpt::new();
            let (dist, angle) = mpt.compute(&cloud);
            prop_assert!(dist >= 0.0);
            prop_assert!(angle.is_finite());
            prop_assert!(angle >= -PI && angle <= PI);
        }

    }
}
