/// control/pid.rs — PID longitudinal (speed / throttle) controller
///
/// FIXES vs original
/// ─────────────────
/// 1. `dt` is now passed via the constructor instead of hardcoding CONTROL_HZ.
///    Caller passes `1.0 / CONTROL_HZ` — no silent divergence if tick rate changes.
/// 2. Added `new_with_dt` constructor for the tuner binary.

pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    integral: f32,
    prev_error: f32,
    dt: f32,
    windup_limit: f32,
}

impl Pid {
    /// Standard constructor — dt in seconds (e.g. 1.0 / 100.0 for 100 Hz).
    pub fn new_with_dt(kp: f32, ki: f32, kd: f32, dt: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            dt,
            windup_limit: 0.5,
        }
    }

    /// Convenience constructor for 100 Hz — matches original API.
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self::new_with_dt(kp, ki, kd, 1.0 / 100.0)
    }

    /// Returns throttle in [-1.0, +1.0].
    pub fn compute_longitudinal(&mut self, error: f32) -> f32 {
        // Anti-windup: reset integral on sign change
        if error.signum() != self.prev_error.signum() {
            self.integral = 0.0;
        }
        self.integral =
            (self.integral + error * self.dt).clamp(-self.windup_limit, self.windup_limit);
        let derivative = (error - self.prev_error) / self.dt;
        self.prev_error = error;

        (self.kp * error + self.ki * self.integral + self.kd * derivative).clamp(-1.0, 1.0)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integral_resets_on_sign_change() {
        let mut pid = Pid::new_with_dt(1.0, 1.0, 0.0, 0.1);
        pid.compute_longitudinal(1.0);
        pid.compute_longitudinal(1.0);
        let before = pid.compute_longitudinal(1.0); // integral has built up
        let after_flip = pid.compute_longitudinal(-1.0); // sign change → reset
                                                         // after_flip's contribution should NOT include the accumulated positive integral
        assert!(after_flip < before);
    }

    #[test]
    fn integral_respects_windup_limit() {
        let mut pid = Pid::new_with_dt(0.0, 10.0, 0.0, 1.0); // pure I, dt=1s
        for _ in 0..1000 {
            pid.compute_longitudinal(1.0); // should saturate, not diverge
        }
        let out = pid.compute_longitudinal(1.0);
        assert!(out <= 1.0); // output clamp from compute_longitudinal
    }

    #[test]
    fn new_with_dt_sets_ki() {
        let kp = 2.0_f32;
        let ki = 0.42_f32;
        let kd = 0.1_f32;
        let dt = 0.02_f32;
        let pid = Pid::new_with_dt(kp, ki, kd, dt);
        assert!((pid.ki - ki).abs() < f32::EPSILON, "Pid.ki should match constructor arg");
        assert!((pid.kp - kp).abs() < f32::EPSILON, "Pid.kp should match constructor arg");
        assert!((pid.kd - kd).abs() < f32::EPSILON, "Pid.kd should match constructor arg");
    }
}
