use serde::{Deserialize, Serialize};
/// sim/model.rs — Vehicle dynamics models
///
/// Trait `VehicleModel` abstracts the ODE so the simulator can swap between:
///
///   KinematicBicycle   — pure geometry, no mass/inertia, good enough for low speed
///   DynamicBicycle     — adds lateral tyre forces (linear), needs identified params
///   IdentifiedModel    — all constants recovered from MCAP recordings via least-squares
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
use std::path::Path;

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

impl Default for KinematicBicycle {
    fn default() -> Self {
        Self {
            wheelbase: 0.33, // ~330 mm — measure your car
            motor_gain: 8.0, // tune or identify
            drag_k: 0.5,
            accel_max: 15.0,
        }
    }
}

impl VehicleModel for KinematicBicycle {
    fn step(&mut self, s: &VehicleState, cmd: &ControlCommand, dt: f32) -> VehicleState {
        let v = s.vx; // scalar speed (kinematic: vy = 0)

        // Longitudinal: throttle → accel → speed
        let accel_raw = cmd.throttle * self.motor_gain - self.drag_k * v * v.abs();
        let accel = accel_raw.clamp(-self.accel_max, self.accel_max);
        let vx_new = (v + accel * dt).max(0.0); // no reverse in kinematic model

        // Steering geometry (Ackermann approximation)
        let beta = (cmd.steering_rad.tan() * 0.5 / self.wheelbase).atan(); // slip angle at CoM
        let yaw_rate = v * cmd.steering_rad.tan() / self.wheelbase;

        let yaw_new = s.yaw_rad + yaw_rate * dt;
        let (sy, cy) = (s.yaw_rad + beta).sin_cos();

        VehicleState {
            x: s.x + v * cy * dt,
            y: s.y + v * sy * dt,
            yaw_rad: yaw_new,
            vx: vx_new,
            vy: 0.0,
            yaw_rate,
        }
    }

    fn name(&self) -> &'static str {
        "kinematic_bicycle"
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  2. Dynamic bicycle model (linear tyre forces)
//     Valid at moderate speed where tyre slip is in the linear region.
//     Needs identified Cf, Cr, Iz.
// ══════════════════════════════════════════════════════════════════════════

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
}

impl Default for DynamicBicycle {
    fn default() -> Self {
        // Placeholder values for a ~1.5 kg RC car — replace after identification
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
        }
    }
}

impl VehicleModel for DynamicBicycle {
    fn step(&mut self, s: &VehicleState, cmd: &ControlCommand, dt: f32) -> VehicleState {
        let vx = s.vx.max(0.05); // avoid division by zero at rest

        // Tyre slip angles (small angle, linear region)
        let alpha_f = cmd.steering_rad - (s.vy + self.lf * s.yaw_rate) / vx;
        let alpha_r = -(s.vy - self.lr * s.yaw_rate) / vx;

        // Lateral forces
        let fy_f = self.cf * alpha_f;
        let fy_r = self.cr * alpha_r;

        // Equations of motion (body frame)
        let accel_x_raw = cmd.throttle * self.motor_gain
            - self.drag_k * vx * vx
            - fy_f * cmd.steering_rad.sin() / self.mass; // small angle: ≈ 0
        let accel_x = accel_x_raw.clamp(-self.accel_max, self.accel_max);

        let accel_y = (fy_f * cmd.steering_rad.cos() + fy_r) / self.mass - vx * s.yaw_rate;
        let yaw_accel = (self.lf * fy_f - self.lr * fy_r) / self.iz;

        // Integrate (Euler — use RK4 for higher fidelity if needed)
        let vx_new = (s.vx + accel_x * dt).max(0.0);
        let vy_new = s.vy + accel_y * dt;
        let yr_new = s.yaw_rate + yaw_accel * dt;
        let yaw_new = s.yaw_rad + s.yaw_rate * dt;

        let v_world = (vx_new * vx_new + vy_new * vy_new).sqrt();
        let (sy, cy) = yaw_new.sin_cos();

        VehicleState {
            x: s.x + (vx_new * cy - vy_new * sy) * dt,
            y: s.y + (vx_new * sy + vy_new * cy) * dt,
            yaw_rad: yaw_new,
            vx: vx_new,
            vy: vy_new,
            yaw_rate: yr_new,
        }
    }

    fn name(&self) -> &'static str {
        "dynamic_bicycle"
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  3. Identified model — all constants loaded from model_params.json
//     Drop-in replacement once `tools/identify.py` has run on MCAP recordings.
// ══════════════════════════════════════════════════════════════════════════

#[derive(Serialize, Deserialize, Debug, Clone)]
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
    /// Identification metadata (informational only)
    pub source_mcap: Option<String>,
    pub rmse_lateral: Option<f32>,
    pub rmse_yaw_rate: Option<f32>,
}

impl Default for IdentifiedParams {
    fn default() -> Self {
        // Falls back to the same defaults as DynamicBicycle
        let d = DynamicBicycle::default();
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
        let raw = std::fs::read_to_string(path)?;
        let p: IdentifiedParams = serde_json::from_str(&raw)?;
        Ok(Self::from_params(p))
    }

    pub fn from_params(p: IdentifiedParams) -> Self {
        let inner = DynamicBicycle {
            wheelbase: p.wheelbase,
            lf: p.lf,
            lr: p.lr,
            mass: p.mass,
            iz: p.iz,
            cf: p.cf,
            cr: p.cr,
            motor_gain: p.motor_gain,
            drag_k: p.drag_k,
            accel_max: p.accel_max,
        };
        Self { inner, params: p }
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
