/// sim/source.rs — SimulatorSource: SensorSource backed by synthetic physics
///
/// Implements the same `SensorSource` trait as `LiveSensorSource` and
/// `ReplaySource` — zero changes needed in the control loop.
///
/// Feedback loop
/// ─────────────
/// The control loop writes actuator commands via `SimActuator`, which stores
/// them in a shared `Arc<Mutex<ControlCommand>>`.  `SimulatorSource::next_snapshot()`
/// reads that cell, steps the physics model, ray-casts synthetic LiDAR + sonar,
/// and returns a `SensorSnapshot` with plausible sensor noise added.
///
/// Usage in main.rs
/// ────────────────
///   let world = World::load("track.json")?;
///   let model = Box::new(KinematicBicycle::default());
///   let (source, actuator) = SimulatorSource::new(world, model, SIM_DT);
///   control_loop(Box::new(source), Box::new(actuator), None).await?;
use std::{
    f32::consts::{PI, TAU},
    sync::{Arc, Mutex},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::Result;

use crate::hal::{ActuatorSink, SensorSource};
use crate::state::SensorSnapshot;
use crate::types::{ImuSample, LidarCloud, LidarPoint};

use super::{
    model::{ControlCommand, VehicleModel, VehicleState},
    noise::SensorNoise,
    world::World,
};

// ── Configuration ──────────────────────────────────────────────────────────

/// Simulated LiDAR: mirror the real bucket layout
/// front ±45° at 1° resolution, sides 45–90° at 5°, rear at 10°
const DIST_MAX_M: f32 = 6.0;

/// Sonar opening half-angle [rad] — approximate HC-SR04 beam
const SONAR_HALF_BEAM: f32 = 0.26; // ~15°

const SONAR_DIRECTIONS: [f32; 3] = [0.0, 0.52, -0.52]; // front, front-left, front-right

// ── Shared command cell ────────────────────────────────────────────────────

type CmdCell = Arc<Mutex<ControlCommand>>;

// ── SimulatorSource ────────────────────────────────────────────────────────

pub struct SimulatorSource {
    world: World,
    model: Box<dyn VehicleModel>,
    state: VehicleState,
    cmd: CmdCell,
    dt: f32,
    tick_us: u64,
    noise: SensorNoise,
    exhausted: bool,
    /// Simulation wall-clock: advance dt per call regardless of real time
    sim_us: u64,
}

impl SimulatorSource {
    /// Returns `(source, actuator)` — give source to the control loop, actuator
    /// replaces `ActuatorFactory::build()` output.
    pub fn new(world: World, model: Box<dyn VehicleModel>, dt: f32) -> (Self, SimActuator) {
        let start = VehicleState {
            x: world.start.x,
            y: world.start.y,
            yaw_rad: world.start.yaw_rad,
            ..Default::default()
        };
        let cmd: CmdCell = Arc::new(Mutex::new(ControlCommand::default()));
        let actuator = SimActuator { cmd: cmd.clone() };
        let source = Self {
            world,
            model,
            state: start,
            cmd,
            dt,
            tick_us: (dt * 1_000_000.0) as u64,
            noise: SensorNoise::default(),
            exhausted: false,
            sim_us: 0,
        };
        (source, actuator)
    }

    // ── Physics step ───────────────────────────────────────────────────────

    fn step(&mut self) {
        let cmd = *self.cmd.lock().unwrap();
        self.state = self.model.step(&self.state, &cmd, self.dt);
        self.sim_us += self.tick_us;
    }

    // ── Synthetic sensors ──────────────────────────────────────────────────

    fn synthesize_lidar(&self) -> LidarCloud {
        let mut points = Vec::with_capacity(144);
        let ts = self.sim_us;

        // Replicate the real bucket layout
        let mut angle = -PI;
        while angle < PI {
            let dtheta = if angle.abs() <= 0.785 {
                // ±45° front
                1.0_f32.to_radians()
            } else if angle.abs() <= PI / 2.0 {
                5.0_f32.to_radians()
            } else {
                10.0_f32.to_radians()
            };

            // World-frame ray direction
            let world_angle = self.state.yaw_rad + angle;
            let dist_clean =
                self.world
                    .raycast(self.state.x, self.state.y, world_angle, DIST_MAX_M);

            if dist_clean < DIST_MAX_M {
                let dist_m = self.noise.lidar(dist_clean);
                let x = dist_m * angle.cos();
                let y = -dist_m * angle.sin(); // car frame: +y left, -sin
                points.push(LidarPoint {
                    x,
                    y,
                    dist_m,
                    angle_rad: angle,
                    timestamp_us: ts,
                });
            }

            angle += dtheta;
        }

        // Sort by angle (required by deskew / MPT)
        points.sort_by(|a, b| a.angle_rad.partial_cmp(&b.angle_rad).unwrap());

        LidarCloud {
            points,
            timestamp_us: ts,
        }
    }

    fn synthesize_sonar(&self) -> [f32; 3] {
        SONAR_DIRECTIONS.map(|rel_angle| {
            let world_angle = self.state.yaw_rad + rel_angle;
            let d = self
                .world
                .raycast(self.state.x, self.state.y, world_angle, 4.0);
            self.noise.sonar(d)
        })
    }

    fn synthesize_imu(&self) -> ImuSample {
        // az ≈ 9.81 (flat), gz from yaw_rate
        ImuSample {
            ax: self.noise.imu_accel(0.0),
            ay: self.noise.imu_accel(0.0),
            az: self.noise.imu_accel(9.806),
            gx: 0.0,
            gy: 0.0,
            gz: self.noise.imu_gyro(self.state.yaw_rate),
            timestamp_us: self.sim_us,
        }
    }

    // ── Crash detection ────────────────────────────────────────────────────

    fn is_crashed(&self) -> bool {
        // Crashed if any wall closer than 0.05 m in any direction
        for angle_deg in (0..360).step_by(30) {
            let a = (angle_deg as f32).to_radians() + self.state.yaw_rad;
            if self.world.raycast(self.state.x, self.state.y, a, 0.15) < 0.05 {
                return true;
            }
        }
        false
    }
}

impl SensorSource for SimulatorSource {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        self.step();

        if self.is_crashed() {
            self.exhausted = true;
            tracing::warn!(
                "sim: crash at ({:.2}, {:.2}) yaw={:.2}",
                self.state.x,
                self.state.y,
                self.state.yaw_rad
            );
        }

        Ok(SensorSnapshot {
            lidar: Arc::new(self.synthesize_lidar()),
            imu: self.synthesize_imu(),
            sonar_m: self.synthesize_sonar(),
            sensor_fault: false,
        })
    }

    fn is_exhausted(&self) -> bool {
        self.exhausted
    }
}

// ── SimActuator ────────────────────────────────────────────────────────────

/// Writes commands into the shared cell that `SimulatorSource` reads.
pub struct SimActuator {
    cmd: CmdCell,
}

impl ActuatorSink for SimActuator {
    fn set_throttle(&mut self, throttle: f32) -> Result<()> {
        self.cmd.lock().unwrap().throttle = throttle.clamp(-1.0, 1.0);
        Ok(())
    }
    fn set_steering(&mut self, angle_rad: f32) -> Result<()> {
        self.cmd.lock().unwrap().steering_rad = angle_rad;
        Ok(())
    }
    fn safe_state(&mut self) -> Result<()> {
        let mut c = self.cmd.lock().unwrap();
        c.throttle = 0.0;
        c.steering_rad = 0.0;
        Ok(())
    }
    fn label(&self) -> &'static str {
        "sim"
    }
}
