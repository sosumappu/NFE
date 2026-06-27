use serde::{Deserialize, Serialize};
/// sim/model.rs — Vehicle dynamics models
///
/// Trait `VehicleModel` abstracts the ODE so the simulator can swap between:
///
///   KinematicBicycle   — pure geometry, no mass/inertia, good enough for low speed
///   DynamicBicycle     — lateral tyre forces, inertia, and actuator dynamics
///   IdentifiedModel    — constants recovered from MCAP recordings via least-squares
///
/// Upgrading path
/// ──────────────
///   1. Start with `KinematicBicycle` — it works immediately with no recording data.
///   2. Record several runs at various speeds.
///   3. Run `tools/identify.py` on the MCAP files → produces `model_params.json`.
///   4. Load `IdentifiedModel::from_json("model_params.json")` — drop-in replacement.
///
/// The `identify.py` script (in tools/) does ordinary least-squares on the
/// lateral acceleration residuals to extract: Cf, Cr (cornering stiffnesses),
/// motor_gain (throttle → longitudinal accel), drag_k (quadratic drag).
use std::{f32::consts::PI, path::Path};

// ── Shared state ───────────────────────────────────────────────────────────

/// Full vehicle state in world frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct VehicleState {
    pub x: f32, // world-frame position [m]
    pub y: f32,
    pub yaw_rad: f32,  // heading [rad], 0 = east
    pub vx: f32,       // body-frame longitudinal speed [m/s]
    pub vy: f32,       // body-frame lateral speed [m/s]  (zero in kinematic model)
    pub yaw_rate: f32, // [rad/s]
    pub ax: f32,       // body-frame longitudinal acceleration [m/s²]
    pub ay: f32,       // body-frame lateral acceleration [m/s²]
}

/// Commands coming from the control loop actuator.
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlCommand {
    pub steering_rad: f32, // front wheel angle [rad], positive = left
    pub throttle: f32,     // normalised [-1, +1]
}

// ── Trait ──────────────────────────────────────────────────────────────────

pub trait VehicleModel: Send {
    fn step(&mut self, state: &VehicleState, cmd: &ControlCommand, dt: f32) -> VehicleState;
    fn name(&self) -> &'static str;
}

// ══════════════════════════════════════════════════════════════════════════
//  1. Kinematic bicycle model
//     Good for v < ~1 m/s or when tyre data is unavailable.
//     No lateral dynamics — vy is always zero.
// ══════════════════════════════════════════════════════════════════════════

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[serde(default)]
pub struct KinematicBicycleParams {
    /// Distance between front and rear axles [m]
    #[serde(alias = "wheelbase_m")]
    pub wheelbase: f32,
    /// Normalised throttle → longitudinal acceleration mapping [m/s² per unit]
    #[serde(alias = "motor_gain_ms2")]
    pub motor_gain: f32,
    /// Quadratic drag coefficient [m/s² per (m/s)²]
    pub drag_k: f32,
    /// Maximum longitudinal acceleration [m/s²]
    #[serde(alias = "accel_max_ms2")]
    pub accel_max: f32,
}

impl Default for KinematicBicycleParams {
    fn default() -> Self {
        Self {
            wheelbase: 0.33, // ~330 mm — measure your car
            motor_gain: 8.0, // tune or identify
            drag_k: 0.5,
            accel_max: 15.0,
        }
    }
}

pub struct KinematicBicycle {
    /// Distance between front and rear axles [m]
    pub wheelbase: f32,
    /// Normalised throttle → longitudinal acceleration mapping [m/s² per unit]
    pub motor_gain: f32,
    /// Quadratic drag coefficient [m/s² per (m/s)²]
    pub drag_k: f32,
    /// Maximum longitudinal acceleration [m/s²]
    pub accel_max: f32,
}

impl KinematicBicycle {
    pub fn from_params(params: KinematicBicycleParams) -> Self {
        Self {
            wheelbase: params.wheelbase,
            motor_gain: params.motor_gain,
            drag_k: params.drag_k,
            accel_max: params.accel_max,
        }
    }

    pub fn params(&self) -> KinematicBicycleParams {
        KinematicBicycleParams {
            wheelbase: self.wheelbase,
            motor_gain: self.motor_gain,
            drag_k: self.drag_k,
            accel_max: self.accel_max,
        }
    }
}

impl Default for KinematicBicycle {
    fn default() -> Self {
        Self::from_params(KinematicBicycleParams::default())
    }
}

impl VehicleModel for KinematicBicycle {
    fn step(&mut self, s: &VehicleState, cmd: &ControlCommand, dt: f32) -> VehicleState {
        if dt <= 0.0 {
            return *s;
        }

        let v = s.vx; // scalar speed (kinematic: vy = 0)

        // Longitudinal: throttle → accel → speed
        let accel_raw = cmd.throttle.clamp(-1.0, 1.0) * self.motor_gain - self.drag_k * v * v.abs();
        let accel = accel_raw.clamp(-self.accel_max, self.accel_max);
        let vx_new = (v + accel * dt).max(0.0); // no reverse in kinematic model

        // Steering geometry (Ackermann approximation)
        let beta = (cmd.steering_rad.tan() * 0.5 / self.wheelbase).atan(); // slip angle at CoM
        let yaw_rate = v * cmd.steering_rad.tan() / self.wheelbase;

        let yaw_new = wrap_angle(s.yaw_rad + yaw_rate * dt);
        let (sy, cy) = (s.yaw_rad + beta).sin_cos();

        VehicleState {
            x: s.x + v * cy * dt,
            y: s.y + v * sy * dt,
            yaw_rad: yaw_new,
            vx: vx_new,
            vy: 0.0,
            yaw_rate,
            ax: accel,
            ay: 0.0,
        }
    }

    fn name(&self) -> &'static str {
        "kinematic_bicycle"
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  2. Dynamic bicycle model
//     Valid at moderate speed where tyre slip is in the linear region.
//     Needs identified Cf, Cr, Iz.
// ══════════════════════════════════════════════════════════════════════════

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[serde(default)]
pub struct ServoParams {
    /// First-order steering lag time constant [s].
    pub tau_s: f32,
    /// Maximum steering slew rate [rad/s].
    pub rate_limit_rad_s: f32,
    /// Deadband half-width between requested and actual steering [rad].
    pub backlash_rad: f32,
}

impl Default for ServoParams {
    fn default() -> Self {
        Self {
            tau_s: 0.05,
            rate_limit_rad_s: 8.0,
            backlash_rad: 0.01,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[serde(default)]
pub struct MotorParams {
    /// First-order motor/ESC lag time constant [s].
    pub tau_s: f32,
    /// Symmetric throttle deadband before torque is applied.
    pub deadband: f32,
    /// Braking acceleration multiplier for negative throttle.
    pub brake_gain: f32,
}

impl Default for MotorParams {
    fn default() -> Self {
        Self {
            tau_s: 0.03,
            deadband: 0.05,
            brake_gain: 1.4,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[serde(default)]
pub struct TyreParams {
    /// Surface friction coefficient used as the lateral force peak scale.
    pub mu: f32,
    /// Simplified Pacejka shape factor; 1.3 is a conservative rubber-tyre start.
    pub pacejka_shape: f32,
    /// Keep linear tyres available for comparison against older simulations.
    pub saturating: bool,
}

impl Default for TyreParams {
    fn default() -> Self {
        Self {
            mu: 1.2,
            pacejka_shape: 1.3,
            saturating: true,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[serde(default)]
pub struct ChassisParams {
    /// Centre-of-gravity height used for longitudinal load transfer [m].
    pub cg_height_m: f32,
}

impl Default for ChassisParams {
    fn default() -> Self {
        Self { cg_height_m: 0.06 }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[serde(default)]
pub struct DynamicBicycleParams {
    #[serde(alias = "wheelbase_m")]
    pub wheelbase: f32, // L  [m]
    #[serde(alias = "lf_m")]
    pub lf: f32, // front axle to CoM [m]
    #[serde(alias = "lr_m")]
    pub lr: f32, // rear  axle to CoM [m]
    #[serde(alias = "mass_kg")]
    pub mass: f32, // kg
    #[serde(alias = "iz_kg_m2")]
    pub iz: f32, // yaw moment of inertia [kg·m²]
    #[serde(alias = "cf_n_per_rad")]
    pub cf: f32, // front cornering stiffness [N/rad]
    #[serde(alias = "cr_n_per_rad")]
    pub cr: f32, // rear  cornering stiffness [N/rad]
    #[serde(alias = "motor_gain_ms2")]
    pub motor_gain: f32, // [m/s² per throttle unit]
    pub drag_k: f32, // [m/s² per (m/s)²]
    #[serde(alias = "accel_max_ms2")]
    pub accel_max: f32,
    pub servo: ServoParams,
    pub motor: MotorParams,
    pub tyre: TyreParams,
    pub chassis: ChassisParams,
}

impl Default for DynamicBicycleParams {
    fn default() -> Self {
        Self {
            wheelbase: 0.33,
            lf: 0.165,
            lr: 0.165,
            mass: 1.5,
            iz: 0.04,
            cf: 12.0, // typical RC tyre, identified value will be better
            cr: 10.0,
            motor_gain: 8.0,
            drag_k: 0.5,
            accel_max: 20.0,
            servo: ServoParams::default(),
            motor: MotorParams::default(),
            tyre: TyreParams::default(),
            chassis: ChassisParams::default(),
        }
    }
}

pub struct DynamicBicycle {
    pub wheelbase: f32,  // L  [m]
    pub lf: f32,         // front axle to CoM [m]
    pub lr: f32,         // rear  axle to CoM [m]
    pub mass: f32,       // kg
    pub iz: f32,         // yaw moment of inertia [kg·m²]
    pub cf: f32,         // front cornering stiffness [N/rad]
    pub cr: f32,         // rear  cornering stiffness [N/rad]
    pub motor_gain: f32, // [m/s² per throttle unit]
    pub drag_k: f32,     // [m/s² per (m/s)²]
    pub accel_max: f32,
    pub servo: ServoParams,
    pub motor: MotorParams,
    pub tyre: TyreParams,
    pub chassis: ChassisParams,
    pub actual_steering_rad: f32,
    pub applied_accel_ms2: f32,
}

impl DynamicBicycle {
    pub fn from_params(params: DynamicBicycleParams) -> Self {
        Self {
            wheelbase: params.wheelbase,
            lf: params.lf,
            lr: params.lr,
            mass: params.mass,
            iz: params.iz,
            cf: params.cf,
            cr: params.cr,
            motor_gain: params.motor_gain,
            drag_k: params.drag_k,
            accel_max: params.accel_max,
            servo: params.servo,
            motor: params.motor,
            tyre: params.tyre,
            chassis: params.chassis,
            actual_steering_rad: 0.0,
            applied_accel_ms2: 0.0,
        }
    }

    pub fn params(&self) -> DynamicBicycleParams {
        DynamicBicycleParams {
            wheelbase: self.wheelbase,
            lf: self.lf,
            lr: self.lr,
            mass: self.mass,
            iz: self.iz,
            cf: self.cf,
            cr: self.cr,
            motor_gain: self.motor_gain,
            drag_k: self.drag_k,
            accel_max: self.accel_max,
            servo: self.servo,
            motor: self.motor,
            tyre: self.tyre,
            chassis: self.chassis,
        }
    }

    fn update_actuators(&mut self, cmd: &ControlCommand, dt: f32) -> DynamicInput {
        let target_steering = finite_or_zero(cmd.steering_rad);
        let steering_error = target_steering - self.actual_steering_rad;
        let effective_error = if steering_error.abs() <= self.servo.backlash_rad.max(0.0) {
            0.0
        } else {
            steering_error - self.servo.backlash_rad.max(0.0) * steering_error.signum()
        };
        let steering_rate = if self.servo.tau_s <= 0.0 {
            effective_error / dt.max(1e-6)
        } else {
            effective_error / self.servo.tau_s
        };
        let rate_limit = self.servo.rate_limit_rad_s.max(0.0);
        let steering_rate = steering_rate.clamp(-rate_limit, rate_limit);
        self.actual_steering_rad += steering_rate * dt;

        let throttle = finite_or_zero(cmd.throttle).clamp(-1.0, 1.0);
        let deadband = self.motor.deadband.clamp(0.0, 0.99);
        let effective_throttle = if throttle.abs() <= deadband {
            0.0
        } else {
            throttle.signum() * (throttle.abs() - deadband) / (1.0 - deadband)
        };
        let gain = if effective_throttle < 0.0 {
            self.motor_gain * self.motor.brake_gain.max(0.0)
        } else {
            self.motor_gain
        };
        let target_accel = effective_throttle * gain;
        if self.motor.tau_s <= 0.0 {
            self.applied_accel_ms2 = target_accel;
        } else {
            let alpha = (dt / self.motor.tau_s).clamp(0.0, 1.0);
            self.applied_accel_ms2 += (target_accel - self.applied_accel_ms2) * alpha;
        }

        DynamicInput {
            steering_rad: self.actual_steering_rad,
            applied_accel_ms2: self.applied_accel_ms2,
        }
    }

    fn normal_loads(&self, mass: f32, accel_x: f32) -> (f32, f32) {
        let wheelbase = self.wheelbase.max((self.lf + self.lr).max(1e-3));
        let h = self.chassis.cg_height_m.max(0.0);
        let g = 9.806_f32;
        let front = mass * (g * self.lr - accel_x * h) / wheelbase;
        let rear = mass * (g * self.lf + accel_x * h) / wheelbase;
        (front.max(0.0), rear.max(0.0))
    }

    fn lateral_force(&self, alpha: f32, stiffness: f32, normal_load: f32) -> f32 {
        if !self.tyre.saturating {
            return stiffness * alpha;
        }
        let peak = (self.tyre.mu.max(0.0) * normal_load.max(0.0)).max(1e-3);
        let shape = self.tyre.pacejka_shape.max(0.1);
        let stiffness_factor = stiffness / (shape * peak);
        peak * (shape * (stiffness_factor * alpha).atan()).sin()
    }

    fn eom(&self, s: &DynamicState, input: DynamicInput) -> DynamicDeriv {
        let vx = s.vx.max(0.05); // avoid division by zero at rest
        let vx_motion = s.vx.max(0.0);
        let steering = input.steering_rad;
        let mass = self.mass.max(1e-3);
        let iz = self.iz.max(1e-6);

        // Tyre slip angles; atan keeps the model bounded outside small angles.
        let alpha_f = steering - ((s.vy + self.lf * s.yaw_rate) / vx).atan();
        let alpha_r = -((s.vy - self.lr * s.yaw_rate) / vx).atan();

        let drive_accel = (input.applied_accel_ms2 - self.drag_k * vx_motion * vx_motion.abs())
            .clamp(-self.accel_max, self.accel_max);
        let (normal_f, normal_r) = self.normal_loads(mass, input.applied_accel_ms2);

        // Lateral forces. The sign convention is positive left.
        let fy_f = self.lateral_force(alpha_f, self.cf, normal_f);
        let fy_r = self.lateral_force(alpha_r, self.cr, normal_r);

        let dvx = drive_accel - fy_f * steering.sin() / mass + s.vy * s.yaw_rate;
        let dvy = (fy_f * steering.cos() + fy_r) / mass - vx_motion * s.yaw_rate;
        let dyaw_rate = (self.lf * fy_f * steering.cos() - self.lr * fy_r) / iz;

        let (sy, cy) = s.yaw.sin_cos();
        DynamicDeriv {
            dvx,
            dvy,
            dyaw_rate,
            dx: vx_motion * cy - s.vy * sy,
            dy: vx_motion * sy + s.vy * cy,
            dyaw: s.yaw_rate,
        }
    }

    fn rk4(&self, s: &DynamicState, input: DynamicInput, dt: f32) -> (DynamicState, DynamicDeriv) {
        let k1 = self.eom(s, input);
        let s2 = add_deriv(s, &k1, dt * 0.5);
        let k2 = self.eom(&s2, input);
        let s3 = add_deriv(s, &k2, dt * 0.5);
        let k3 = self.eom(&s3, input);
        let s4 = add_deriv(s, &k3, dt);
        let k4 = self.eom(&s4, input);

        let next = add_weighted(s, &k1, &k2, &k3, &k4, dt);
        let final_deriv = self.eom(&next, input);
        (next, final_deriv)
    }
}

impl Default for DynamicBicycle {
    fn default() -> Self {
        // Placeholder values for a ~1.5 kg RC car — replace after identification
        Self::from_params(DynamicBicycleParams::default())
    }
}

impl VehicleModel for DynamicBicycle {
    fn step(&mut self, s: &VehicleState, cmd: &ControlCommand, dt: f32) -> VehicleState {
        if dt <= 0.0 {
            return *s;
        }

        let input = self.update_actuators(cmd, dt);
        let state = DynamicState::from(*s);
        let (mut next, deriv) = self.rk4(&state, input, dt);
        if next.vx < 0.0 {
            next.vx = 0.0;
        }
        next.yaw = wrap_angle(next.yaw);

        VehicleState {
            x: next.x,
            y: next.y,
            yaw_rad: next.yaw,
            vx: next.vx,
            vy: next.vy,
            yaw_rate: next.yaw_rate,
            ax: deriv.dvx,
            ay: deriv.dvy,
        }
    }

    fn name(&self) -> &'static str {
        "dynamic_bicycle"
    }
}

#[derive(Debug, Clone, Copy)]
struct DynamicInput {
    steering_rad: f32,
    applied_accel_ms2: f32,
}

#[derive(Debug, Clone, Copy)]
struct DynamicState {
    x: f32,
    y: f32,
    yaw: f32,
    vx: f32,
    vy: f32,
    yaw_rate: f32,
}

impl From<VehicleState> for DynamicState {
    fn from(s: VehicleState) -> Self {
        Self {
            x: s.x,
            y: s.y,
            yaw: s.yaw_rad,
            vx: s.vx,
            vy: s.vy,
            yaw_rate: s.yaw_rate,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct DynamicDeriv {
    dx: f32,
    dy: f32,
    dyaw: f32,
    dvx: f32,
    dvy: f32,
    dyaw_rate: f32,
}

fn add_deriv(s: &DynamicState, d: &DynamicDeriv, scale: f32) -> DynamicState {
    DynamicState {
        x: s.x + d.dx * scale,
        y: s.y + d.dy * scale,
        yaw: s.yaw + d.dyaw * scale,
        vx: s.vx + d.dvx * scale,
        vy: s.vy + d.dvy * scale,
        yaw_rate: s.yaw_rate + d.dyaw_rate * scale,
    }
}

fn add_weighted(
    s: &DynamicState,
    k1: &DynamicDeriv,
    k2: &DynamicDeriv,
    k3: &DynamicDeriv,
    k4: &DynamicDeriv,
    dt: f32,
) -> DynamicState {
    let scale = dt / 6.0;
    DynamicState {
        x: s.x + (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx) * scale,
        y: s.y + (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy) * scale,
        yaw: s.yaw + (k1.dyaw + 2.0 * k2.dyaw + 2.0 * k3.dyaw + k4.dyaw) * scale,
        vx: s.vx + (k1.dvx + 2.0 * k2.dvx + 2.0 * k3.dvx + k4.dvx) * scale,
        vy: s.vy + (k1.dvy + 2.0 * k2.dvy + 2.0 * k3.dvy + k4.dvy) * scale,
        yaw_rate: s.yaw_rate
            + (k1.dyaw_rate + 2.0 * k2.dyaw_rate + 2.0 * k3.dyaw_rate + k4.dyaw_rate) * scale,
    }
}

fn finite_or_zero(v: f32) -> f32 {
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

fn wrap_angle(a: f32) -> f32 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

// ══════════════════════════════════════════════════════════════════════════
//  3. Identified model — constants loaded from model_params.json
//     Drop-in replacement once `tools/identify.py` has run on MCAP recordings.
// ══════════════════════════════════════════════════════════════════════════

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct IdentifiedParams {
    pub wheelbase: f32,
    pub lf: f32,
    pub lr: f32,
    pub mass: f32,
    pub iz: f32,
    pub cf: f32,
    pub cr: f32,
    pub motor_gain: f32,
    pub drag_k: f32,
    pub accel_max: f32,
    pub servo: Option<ServoParams>,
    pub motor: Option<MotorParams>,
    pub tyre: Option<TyreParams>,
    pub chassis: Option<ChassisParams>,
    /// Identification metadata (informational only)
    pub source_mcap: Option<String>,
    pub rmse_lateral: Option<f32>,
    pub rmse_yaw_rate: Option<f32>,
}

impl Default for IdentifiedParams {
    fn default() -> Self {
        // Falls back to the same defaults as DynamicBicycle
        let d = DynamicBicycleParams::default();
        Self {
            wheelbase: d.wheelbase,
            lf: d.lf,
            lr: d.lr,
            mass: d.mass,
            iz: d.iz,
            cf: d.cf,
            cr: d.cr,
            motor_gain: d.motor_gain,
            drag_k: d.drag_k,
            accel_max: d.accel_max,
            servo: None,
            motor: None,
            tyre: None,
            chassis: None,
            source_mcap: None,
            rmse_lateral: None,
            rmse_yaw_rate: None,
        }
    }
}

pub struct IdentifiedModel {
    inner: DynamicBicycle,
    pub params: IdentifiedParams,
}

impl IdentifiedModel {
    pub fn from_json(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        Self::from_json_with_base(path, DynamicBicycleParams::default())
    }

    pub fn from_json_with_base(
        path: impl AsRef<Path>,
        base: DynamicBicycleParams,
    ) -> anyhow::Result<Self> {
        let raw = std::fs::read_to_string(path)?;
        let p: IdentifiedParams = serde_json::from_str(&raw)?;
        Ok(Self::from_params_with_base(p, base))
    }

    pub fn from_params(p: IdentifiedParams) -> Self {
        Self::from_params_with_base(p, DynamicBicycleParams::default())
    }

    pub fn from_params_with_base(p: IdentifiedParams, mut base: DynamicBicycleParams) -> Self {
        let mut params = p;
        base.wheelbase = params.wheelbase;
        base.lf = params.lf;
        base.lr = params.lr;
        base.mass = params.mass;
        base.iz = params.iz;
        base.cf = params.cf;
        base.cr = params.cr;
        base.motor_gain = params.motor_gain;
        base.drag_k = params.drag_k;
        base.accel_max = params.accel_max;
        if let Some(servo) = params.servo {
            base.servo = servo;
        } else {
            params.servo = Some(base.servo);
        }
        if let Some(motor) = params.motor {
            base.motor = motor;
        } else {
            params.motor = Some(base.motor);
        }
        if let Some(tyre) = params.tyre {
            base.tyre = tyre;
        } else {
            params.tyre = Some(base.tyre);
        }
        if let Some(chassis) = params.chassis {
            base.chassis = chassis;
        } else {
            params.chassis = Some(base.chassis);
        }
        let inner = DynamicBicycle::from_params(base);
        Self { inner, params }
    }

    /// Serialise current params back to JSON (after in-run CMA-ES tuning).
    pub fn save(&self, path: impl AsRef<Path>) -> anyhow::Result<()> {
        let json = serde_json::to_string_pretty(&self.params)?;
        std::fs::write(path, json)?;
        Ok(())
    }
}

impl VehicleModel for IdentifiedModel {
    fn step(&mut self, s: &VehicleState, cmd: &ControlCommand, dt: f32) -> VehicleState {
        self.inner.step(s, cmd, dt)
    }
    fn name(&self) -> &'static str {
        "identified"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dynamic_servo_rate_limits_steering() {
        let mut model = DynamicBicycle::from_params(DynamicBicycleParams {
            servo: ServoParams {
                tau_s: 0.01,
                rate_limit_rad_s: 1.0,
                backlash_rad: 0.0,
            },
            ..DynamicBicycleParams::default()
        });
        let state = VehicleState {
            vx: 1.0,
            ..VehicleState::default()
        };
        let cmd = ControlCommand {
            steering_rad: 1.0,
            throttle: 0.0,
        };
        let _ = model.step(&state, &cmd, 0.1);
        assert!(model.actual_steering_rad <= 0.101);
    }

    #[test]
    fn dynamic_motor_deadband_holds_applied_accel_zero() {
        let mut model = DynamicBicycle::from_params(DynamicBicycleParams {
            motor: MotorParams {
                tau_s: 0.0,
                deadband: 0.1,
                brake_gain: 1.0,
            },
            ..DynamicBicycleParams::default()
        });
        let state = VehicleState::default();
        let cmd = ControlCommand {
            steering_rad: 0.0,
            throttle: 0.05,
        };
        let _ = model.step(&state, &cmd, 0.01);
        assert_eq!(model.applied_accel_ms2, 0.0);
    }

    #[test]
    fn dynamic_rk4_accelerates_forward() {
        let mut model = DynamicBicycle::from_params(DynamicBicycleParams {
            motor: MotorParams {
                tau_s: 0.0,
                deadband: 0.0,
                brake_gain: 1.0,
            },
            ..DynamicBicycleParams::default()
        });
        let state = VehicleState::default();
        let cmd = ControlCommand {
            steering_rad: 0.0,
            throttle: 1.0,
        };
        let next = model.step(&state, &cmd, 0.1);
        assert!(next.vx > 0.0);
        assert!(next.ax > 0.0);
    }

    #[test]
    fn saturating_tyres_bound_lateral_force_by_normal_load() {
        let model = DynamicBicycle::from_params(DynamicBicycleParams {
            cf: 1_000.0,
            tyre: TyreParams {
                mu: 1.0,
                pacejka_shape: 1.3,
                saturating: true,
            },
            ..DynamicBicycleParams::default()
        });
        let force = model.lateral_force(2.0, model.cf, 5.0);
        assert!(force.abs() <= 5.01);
    }

    #[test]
    fn identified_params_accept_legacy_json() {
        let raw = r#"{
            "wheelbase": 0.33,
            "lf": 0.16,
            "lr": 0.17,
            "mass": 1.5,
            "iz": 0.04,
            "cf": 12.0,
            "cr": 10.0,
            "motor_gain": 8.0,
            "drag_k": 0.5,
            "accel_max": 20.0
        }"#;
        let params: IdentifiedParams = serde_json::from_str(raw).unwrap();
        assert!(params.servo.is_none());
        let model = IdentifiedModel::from_params(params);
        assert_eq!(model.name(), "identified");
    }
}
