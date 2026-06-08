use crate::types::LidarCloudView;
use std::f32::consts::{PI, TAU};

pub struct SpeedPlanner {
    v_max: f32,
    k_dist: f32,    // gain for waypoint-distance term
    k_heading: f32, // curvature of the heading penalty parabola
}

impl SpeedPlanner {
    pub fn new(v_max: f32, k_dist: f32, k_heading: f32) -> Self {
        Self {
            v_max,
            k_dist,
            k_heading,
        }
    }

    /// 60° cone around the steering angle, weighted-average inverse-square distance → proximity factor [0,1]
    /// 1 = clear, 0 = obstacle very close
    fn proximity_factor(&self, cloud: &LidarCloudView, steering_angle: f32) -> f32 {
        let half_arc = 30.0_f32.to_radians();
        let mut num = 0.0;
        let mut den = 0.0;
        for p in cloud.points {
            if ((p.angle_rad - steering_angle + PI).rem_euclid(TAU) - PI).abs() <= half_arc {
                // TODO: Smooth the weight values
                let w = 1.0 / (p.angle_rad.abs() + 0.05); // weight: closer to waypoint = more important
                num += w / (p.dist_m * p.dist_m).max(0.01);
                den += w;
            }
        }
        if den == 0.0 {
            return 1.0;
        }
        let danger = num / den;
        (1.0 / (1.0 + danger)).clamp(0.0, 1.0)
    }

    /// Distance to waypoint factor — ramps speed down on approach
    fn distance_factor(&self, dist_to_waypoint_m: f32) -> f32 {
        (dist_to_waypoint_m * self.k_dist).clamp(0.0, 1.0)
    }

    /// Negative parabola on heading error: 1.0 at zero error, falling off as error grows
    fn heading_factor(&self, heading_err_rad: f32) -> f32 {
        (1.0 - self.k_heading * heading_err_rad * heading_err_rad).clamp(0.0, 1.0)
    }

    /// Combined target speed in m/s
    pub fn compute(
        &self,
        cloud: &LidarCloudView,
        dist_to_waypoint_m: f32,
        heading_err_rad: f32,
    ) -> f32 {
        self.v_max
            * self.proximity_factor(cloud, heading_err_rad)
            * self.distance_factor(dist_to_waypoint_m)
            * self.heading_factor(heading_err_rad)
    }
}
