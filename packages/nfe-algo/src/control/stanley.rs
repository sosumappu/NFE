use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, Tunable)]
#[serde(default)]
pub struct StanleyParams {
    #[param(1.0..20.00, default = 2.5)]
    pub k_cross_track: f32,
    #[param(0.0..20.00, default = 2.0)]
    pub softening_speed_ms: f32,
    #[param(0.15..0.7, default = 0.28)]
    pub max_steering_rad: f32,
}

impl Default for StanleyParams {
    fn default() -> Self {
        Self {
            k_cross_track: 2.5,
            softening_speed_ms: 1.0,
            max_steering_rad: 0.38,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Stanley {
    params: StanleyParams,
}

impl Stanley {
    pub fn new(params: StanleyParams) -> Self {
        Self { params }
    }

    pub fn compute(&self, speed_ms: f32, lateral_error_m: f32, heading_error_rad: f32) -> f32 {
        (heading_error_rad
            + (self.params.k_cross_track * lateral_error_m)
                .atan2(speed_ms.abs() + self.params.softening_speed_ms))
        .clamp(-self.params.max_steering_rad, self.params.max_steering_rad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn higher_speed_reduce_crosstrack() {
        let stanley = Stanley::new(StanleyParams::default());
        let out1 = stanley.compute(1.0, 0.3, 0.3);
        let out2 = stanley.compute(5.0, 0.3, 0.3);
        println!("{} out1, {} out2", out1, out2);
        assert!(out1 >= out2);
    }

    #[test]
    fn track_to_the_left_positive_steering() {
        let stanley = Stanley::new(StanleyParams::default());
        let out = stanley.compute(1.0, 1.0, 1.0);
        assert!(out >= 0.0);
    }

    #[test]
    fn track_to_the_right_positive_steering() {
        let stanley = Stanley::new(StanleyParams::default());
        let out = stanley.compute(1.0, -1.0, -1.0);
        assert!(out <= 0.0);
    }
}
