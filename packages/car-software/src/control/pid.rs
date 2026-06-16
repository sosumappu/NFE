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
