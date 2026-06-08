use crate::types::LidarCloudView;

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
