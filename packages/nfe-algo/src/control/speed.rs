use nfe_core::control::CorridorEstimate;
use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct SpeedParams {
    #[param(0.1..10.0, default = 1.8)]
    pub v_max: f32,
    #[param(0.0..5.0, default = 0.35)]
    pub v_min: f32,
    #[param(0.0..20.0, default = 2.0)]
    pub k_heading: f32,
    #[param(0.0..20.0, default = 2.0)]
    pub k_lateral: f32,
    #[param(0.05..10.0, default = 3.0)]
    pub obstacle_slowdown_m: f32,
    #[param(0.1..20.0, default = 3.0)]
    pub a_lat_max_ms2: f32,
    #[param(0.0..20.0, default = 1.5)]
    pub accel_limit_ms2: f32,
    #[param(0.0..30.0, default = 5.0)]
    pub decel_limit_ms2: f32,
}

impl Default for SpeedParams {
    fn default() -> Self {
        Self {
            v_max: 1.8,
            v_min: 0.35,
            k_heading: 2.0,
            k_lateral: 2.0,
            obstacle_slowdown_m: 3.0,
            a_lat_max_ms2: 3.0,
            accel_limit_ms2: 1.5,
            decel_limit_ms2: 5.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SpeedPlanner {
    params: SpeedParams,
    prev_target_speed_ms: f32,
}

impl SpeedPlanner {
    pub fn new(params: SpeedParams) -> Self {
        Self {
            params,
            prev_target_speed_ms: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.prev_target_speed_ms = 0.0;
    }

    pub fn compute(&mut self, corridor: Option<&CorridorEstimate>, dt_s: f32) -> f32 {
        let Some(c) = corridor else {
            self.prev_target_speed_ms = 0.0;
            return 0.0;
        };

        let raw_target = self.raw_target_speed(c);
        self.rate_limit(raw_target, dt_s)
    }

    fn raw_target_speed(&self, c: &CorridorEstimate) -> f32 {
        let v_max = self.params.v_max.max(0.0);
        if v_max <= f32::EPSILON {
            return 0.0;
        }

        let v_min = self.params.v_min.clamp(0.0, v_max);
        let heading_cap =
            error_speed_cap(v_max, self.params.k_heading, c.heading_error_rad.powi(2));
        let lateral_cap = error_speed_cap(v_max, self.params.k_lateral, c.lateral_error_m.abs());
        let curvature_cap =
            curvature_speed_cap(v_max, self.params.a_lat_max_ms2, c.curvature_m_inv);

        let path_cap = heading_cap.min(lateral_cap).min(curvature_cap);
        let path_target = if path_cap > 0.0 {
            path_cap.max(v_min)
        } else {
            0.0
        };
        let obstacle_cap =
            obstacle_speed_cap(v_max, self.params.obstacle_slowdown_m, c.nearest_obstacle_m);
        let confidence_cap = v_max * c.confidence.clamp(0.1, 1.0);

        path_target
            .min(obstacle_cap)
            .min(confidence_cap)
            .clamp(0.0, v_max)
    }

    fn rate_limit(&mut self, target_speed_ms: f32, dt_s: f32) -> f32 {
        let target_speed_ms = finite_non_negative(target_speed_ms);
        if dt_s <= 0.0 || !dt_s.is_finite() {
            self.prev_target_speed_ms = target_speed_ms;
            return target_speed_ms;
        }

        let prev = finite_non_negative(self.prev_target_speed_ms);
        let limit = if target_speed_ms >= prev {
            self.params.accel_limit_ms2
        } else {
            self.params.decel_limit_ms2
        }
        .max(0.0);
        let max_delta = limit * dt_s;
        let next = if max_delta <= 0.0 {
            prev
        } else {
            prev + (target_speed_ms - prev).clamp(-max_delta, max_delta)
        };

        self.prev_target_speed_ms = next;
        next
    }
}

fn error_speed_cap(v_max: f32, gain: f32, error: f32) -> f32 {
    if gain <= 0.0 || error <= 0.0 || !error.is_finite() {
        return v_max;
    }

    v_max / (1.0 + gain * error)
}

fn curvature_speed_cap(v_max: f32, a_lat_max_ms2: f32, curvature_m_inv: f32) -> f32 {
    let curvature = curvature_m_inv.abs();
    if curvature <= 1.0e-4 || !curvature.is_finite() || a_lat_max_ms2 <= 0.0 {
        return v_max;
    }

    (a_lat_max_ms2 / curvature).sqrt().min(v_max)
}

fn obstacle_speed_cap(v_max: f32, slowdown_m: f32, nearest_obstacle_m: f32) -> f32 {
    if !nearest_obstacle_m.is_finite() {
        return v_max;
    }

    let slowdown_m = slowdown_m.max(f32::EPSILON);
    v_max * (nearest_obstacle_m / slowdown_m).clamp(0.0, 1.0)
}

fn finite_non_negative(value: f32) -> f32 {
    if value.is_finite() {
        value.max(0.0)
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corridor() -> CorridorEstimate {
        CorridorEstimate {
            nearest_obstacle_m: f32::INFINITY,
            confidence: 1.0,
            ..Default::default()
        }
    }

    fn unrestricted_params() -> SpeedParams {
        SpeedParams {
            v_max: 5.0,
            v_min: 0.0,
            k_heading: 0.0,
            k_lateral: 0.0,
            obstacle_slowdown_m: 3.0,
            a_lat_max_ms2: 4.0,
            accel_limit_ms2: 100.0,
            decel_limit_ms2: 100.0,
        }
    }

    #[test]
    fn curvature_caps_speed_from_lateral_accel() {
        let mut planner = SpeedPlanner::new(unrestricted_params());
        let mut c = corridor();
        c.curvature_m_inv = 2.0;

        let target = planner.compute(Some(&c), 1.0);

        assert!((target - 2.0_f32.sqrt()).abs() < 1.0e-4, "target={target}");
    }

    #[test]
    fn infinite_obstacle_distance_does_not_slow() {
        let mut planner = SpeedPlanner::new(unrestricted_params());
        let c = corridor();

        let target = planner.compute(Some(&c), 1.0);

        assert!((target - 5.0).abs() < 1.0e-4, "target={target}");
    }

    #[test]
    fn finite_obstacle_distance_can_stop_below_creep_speed() {
        let mut planner = SpeedPlanner::new(SpeedParams {
            v_min: 0.5,
            ..unrestricted_params()
        });
        let mut c = corridor();
        c.nearest_obstacle_m = 0.0;

        let target = planner.compute(Some(&c), 1.0);

        assert_eq!(target, 0.0);
    }

    #[test]
    fn target_speed_is_accel_limited() {
        let mut planner = SpeedPlanner::new(SpeedParams {
            accel_limit_ms2: 1.0,
            ..unrestricted_params()
        });
        let c = corridor();

        let target = planner.compute(Some(&c), 0.1);

        assert!((target - 0.1).abs() < 1.0e-6, "target={target}");
    }
}
