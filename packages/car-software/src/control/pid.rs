/// control/pid.rs — PID longitudinal (speed / throttle) controller
///
/// Computes a throttle command [-1, +1] from speed error.
/// Anti-windup: integral is clamped and reset on sign change.

// TODO: pass CONTROL_HZ as dt in  the PID constructor to avoid duplication
const CONTROL_HZ: f32 = 100.0;

pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    integral:   f32,
    prev_error: f32,
    dt:         f32,
    windup_limit: f32, // clamping pour l'intégral
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp, ki, kd,
            integral:     0.0,
            prev_error:   0.0,
            dt:           1.0 / CONTROL_HZ,
            windup_limit: 0.5,
        }
    }

    /// return [-1.0, +1.0].
    pub fn compute_longitudinal(&mut self, error: f32) -> f32 {
        // Anti-windup: reset integral on sign change
        if error.signum() != self.prev_error.signum() {
            self.integral = 0.0;
        }
        self.integral = (self.integral + error * self.dt).clamp(
            -self.windup_limit, self.windup_limit
        );
        let derivative = (error - self.prev_error) / self.dt;
        self.prev_error = error;

        (self.kp * error + self.ki * self.integral + self.kd * derivative)
            .clamp(-1.0, 1.0)
    }

    pub fn reset(&mut self) {
        self.integral   = 0.0;
        self.prev_error = 0.0;
    }
}
