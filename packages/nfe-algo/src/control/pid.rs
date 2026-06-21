use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct PidParams {
    #[param(0.0..20.0, default = 1.5)]
    pub kp: f32,
    #[param(0.0..10.0, default = 0.05)]
    pub ki: f32,
    #[param(0.0..10.0, default = 0.2)]
    pub kd: f32,
    #[param(0.0..10.0, default = 0.5)]
    pub windup_limit: f32,
    #[param(0.05..1.0, default = 1.0)]
    pub max_throttle: f32,
}

impl Default for PidParams {
    fn default() -> Self {
        Self {
            kp: 1.5,
            ki: 0.05,
            kd: 0.2,
            windup_limit: 0.5,
            max_throttle: 1.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Pid {
    params: PidParams,
    integral: f32,
    prev_error: f32,
}

impl Pid {
    pub fn new(params: PidParams) -> Self {
        Self {
            params,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    pub fn compute(&mut self, error: f32, dt_s: f32) -> f32 {
        if dt_s <= 0.0 {
            return 0.0;
        }
        if error.signum() != self.prev_error.signum() {
            self.integral = 0.0;
        }
        self.integral = (self.integral + error * dt_s)
            .clamp(-self.params.windup_limit, self.params.windup_limit);
        let derivative = (error - self.prev_error) / dt_s;
        self.prev_error = error;
        (self.params.kp * error + self.params.ki * self.integral + self.params.kd * derivative)
            .clamp(-self.params.max_throttle, self.params.max_throttle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resets_integral_on_sign_change() {
        let mut pid = Pid::new(PidParams::default());
        let _ = pid.compute(1.0, 0.1);
        let _ = pid.compute(1.0, 0.1);
        let before = pid.integral;
        let _ = pid.compute(-1.0, 0.1);
        assert!(pid.integral.abs() < before.abs());
    }
}
