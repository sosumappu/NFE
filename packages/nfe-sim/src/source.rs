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
    collections::VecDeque,
    f32::consts::PI,
    sync::{Arc, Mutex},
};

use anyhow::Result;

use nfe_core::control::ControlOutput;
use nfe_core::estimation::ImuSample;
use nfe_core::io::{ActuatorSink, SensorSource};
use nfe_core::sensors::{LidarCloud, LidarPoint, SensorSnapshot};

use crate::{
    model::{ControlCommand, VehicleModel, VehicleState},
    noise::SensorNoise,
    world::{VehicleFootprintParams, World},
};

// ── Configuration ──────────────────────────────────────────────────────────

/// Simulated LiDAR: mirror the real bucket layout
/// front ±45° at 1° resolution, sides 45–90° at 5°, rear at 10°
const DIST_MAX_M: f32 = 6.0;

const SONAR_DIRECTIONS: [f32; 3] = [0.0, 0.52, -0.52]; // front, front-left, front-right

// ── Shared command cell ────────────────────────────────────────────────────

type CmdCell = Arc<Mutex<ControlCommand>>;

#[derive(Clone, Copy, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct LatencyParams {
    /// Delay between command publication and model application.
    pub latency_us: u64,
}

impl Default for LatencyParams {
    fn default() -> Self {
        Self { latency_us: 0 }
    }
}

#[derive(Debug)]
struct LatencyBuffer {
    queue: VecDeque<(ControlCommand, u64)>,
    latency_us: u64,
}

impl LatencyBuffer {
    fn new(params: LatencyParams) -> Self {
        Self {
            queue: VecDeque::new(),
            latency_us: params.latency_us,
        }
    }

    fn push(&mut self, cmd: ControlCommand, now_us: u64) {
        self.queue
            .push_back((cmd, now_us.saturating_add(self.latency_us)));
    }

    fn poll_due(&mut self, now_us: u64) -> Option<ControlCommand> {
        let mut latest = None;
        while self
            .queue
            .front()
            .map(|(_, apply_at_us)| *apply_at_us <= now_us)
            .unwrap_or(false)
        {
            latest = self.queue.pop_front().map(|(cmd, _)| cmd);
        }
        latest
    }
}

// ── SimulatorSource ────────────────────────────────────────────────────────

pub struct SimulatorSource {
    world: World,
    model: Box<dyn VehicleModel>,
    state: VehicleState,
    cmd: CmdCell,
    applied_cmd: ControlCommand,
    latency: LatencyBuffer,
    footprint: VehicleFootprintParams,
    dt: f32,
    tick_us: u64,
    noise: SensorNoise,
    exhausted: bool,
    exhaustion_reason: Option<String>,
    /// Simulation wall-clock: advance dt per call regardless of real time
    sim_us: u64,
}

impl SimulatorSource {
    /// Returns `(source, actuator)` — give source to the control loop, actuator
    /// replaces `ActuatorFactory::build()` output.
    pub fn new(world: World, model: Box<dyn VehicleModel>, dt: f32) -> (Self, SimActuator) {
        Self::new_with_noise_latency_and_footprint(
            world,
            model,
            dt,
            SensorNoise::default(),
            LatencyParams::default(),
            VehicleFootprintParams::default(),
        )
    }

    pub fn new_with_latency(
        world: World,
        model: Box<dyn VehicleModel>,
        dt: f32,
        latency: LatencyParams,
    ) -> (Self, SimActuator) {
        Self::new_with_noise_latency_and_footprint(
            world,
            model,
            dt,
            SensorNoise::default(),
            latency,
            VehicleFootprintParams::default(),
        )
    }

    pub fn new_with_latency_and_footprint(
        world: World,
        model: Box<dyn VehicleModel>,
        dt: f32,
        latency: LatencyParams,
        footprint: VehicleFootprintParams,
    ) -> (Self, SimActuator) {
        Self::new_with_noise_latency_and_footprint(
            world,
            model,
            dt,
            SensorNoise::default(),
            latency,
            footprint,
        )
    }

    pub fn new_with_seed(
        world: World,
        model: Box<dyn VehicleModel>,
        dt: f32,
        seed: u64,
    ) -> (Self, SimActuator) {
        Self::new_with_noise_latency_and_footprint(
            world,
            model,
            dt,
            SensorNoise::with_seed(seed),
            LatencyParams::default(),
            VehicleFootprintParams::default(),
        )
    }

    pub fn new_with_seed_and_latency(
        world: World,
        model: Box<dyn VehicleModel>,
        dt: f32,
        seed: u64,
        latency: LatencyParams,
    ) -> (Self, SimActuator) {
        Self::new_with_noise_latency_and_footprint(
            world,
            model,
            dt,
            SensorNoise::with_seed(seed),
            latency,
            VehicleFootprintParams::default(),
        )
    }

    pub fn new_with_seed_latency_and_footprint(
        world: World,
        model: Box<dyn VehicleModel>,
        dt: f32,
        seed: u64,
        latency: LatencyParams,
        footprint: VehicleFootprintParams,
    ) -> (Self, SimActuator) {
        Self::new_with_noise_latency_and_footprint(
            world,
            model,
            dt,
            SensorNoise::with_seed(seed),
            latency,
            footprint,
        )
    }

    fn new_with_noise_latency_and_footprint(
        world: World,
        model: Box<dyn VehicleModel>,
        dt: f32,
        noise: SensorNoise,
        latency: LatencyParams,
        footprint: VehicleFootprintParams,
    ) -> (Self, SimActuator) {
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
            applied_cmd: ControlCommand::default(),
            latency: LatencyBuffer::new(latency),
            footprint,
            dt,
            tick_us: (dt * 1_000_000.0) as u64,
            noise,
            exhausted: false,
            exhaustion_reason: None,
            sim_us: 0,
        };
        (source, actuator)
    }

    // ── Physics step ───────────────────────────────────────────────────────

    fn step(&mut self) {
        let requested = *self.cmd.lock().unwrap();
        self.latency.push(requested, self.sim_us);
        if let Some(cmd) = self.latency.poll_due(self.sim_us) {
            self.applied_cmd = cmd;
        }
        self.state = self.model.step(&self.state, &self.applied_cmd, self.dt);
        self.sim_us += self.tick_us;
    }

    // ── Synthetic sensors ──────────────────────────────────────────────────

    fn synthesize_lidar(&mut self) -> LidarCloud {
        let mut points = Vec::with_capacity(144);
        let ts = self.sim_us;

        // Replicate the real bucket layout
        let mut angle = -PI;
        while angle < PI {
            let dtheta = if angle.abs() <= 0.785 {
                // ±45° front
                1.0_f32.to_radians()
            } else if angle.abs() <= PI / 2.0 {
                1.0_f32.to_radians()
            } else {
                10.0_f32.to_radians()
            };

            // World-frame ray direction. Simulated angles use the standard
            // car-local convention: +x forward, +y left, positive yaw left.
            let world_angle = self.state.yaw_rad + angle;
            let dist_clean =
                self.world
                    .raycast(self.state.x, self.state.y, world_angle, DIST_MAX_M);

            if dist_clean < DIST_MAX_M {
                let dist_m = self.noise.lidar(dist_clean);
                let x = dist_m * angle.cos();
                let y = dist_m * angle.sin();
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

    fn synthesize_sonar(&mut self) -> [f32; 3] {
        SONAR_DIRECTIONS.map(|rel_angle| {
            let world_angle = self.state.yaw_rad + rel_angle;
            let d = self
                .world
                .raycast(self.state.x, self.state.y, world_angle, 4.0);
            self.noise.sonar(d)
        })
    }

    fn synthesize_imu(&mut self) -> ImuSample {
        // The estimators consume body-frame acceleration derivatives, so the
        // simulator publishes the model acceleration rather than wall-frame
        // finite differences.
        ImuSample {
            ax: self.noise.imu_accel(self.state.ax),
            ay: self.noise.imu_accel(self.state.ay),
            az: self.noise.imu_accel(9.806),
            gx: 0.0,
            gy: 0.0,
            gz: self.noise.imu_gyro(self.state.yaw_rate),
            timestamp_us: self.sim_us,
        }
    }

    // ── Crash detection ────────────────────────────────────────────────────

    fn is_crashed(&self) -> bool {
        self.world.footprint_intersects_wall(
            self.state.x,
            self.state.y,
            self.state.yaw_rad,
            self.footprint,
        )
    }
}

impl SensorSource for SimulatorSource {
    fn next_snapshot(&mut self) -> Result<Option<SensorSnapshot>> {
        self.step();

        if self.is_crashed() {
            self.exhausted = true;
            let reason = format!(
                "sim crash at ({:.2}, {:.2}) yaw={:.2}",
                self.state.x, self.state.y, self.state.yaw_rad
            );
            self.exhaustion_reason = Some(reason.clone());
            tracing::warn!("{reason}");
        }

        if self.exhausted {
            return Ok(None);
        }

        Ok(Some(SensorSnapshot {
            lidar: self.synthesize_lidar(),
            imu: self.synthesize_imu(),
            sonar_m: self.synthesize_sonar(),
            sensor_fault: false,
            start_line_crossed: false,
        }))
    }
}

impl SimulatorSource {
    pub fn is_exhausted(&self) -> bool {
        self.exhausted
    }

    pub fn exhaustion_reason(&self) -> Option<&str> {
        self.exhaustion_reason.as_deref()
    }

    pub fn vehicle_state(&self) -> VehicleState {
        self.state
    }

    pub fn command(&self) -> ControlCommand {
        self.applied_cmd
    }

    pub fn footprint(&self) -> VehicleFootprintParams {
        self.footprint
    }

    pub fn timestamp_us(&self) -> u64 {
        self.sim_us
    }

    pub fn world(&self) -> &World {
        &self.world
    }
}

// ── SimActuator ────────────────────────────────────────────────────────────

/// Writes commands into the shared cell that `SimulatorSource` reads.
pub struct SimActuator {
    cmd: CmdCell,
}

impl ActuatorSink for SimActuator {
    fn apply(&mut self, output: &ControlOutput) -> Result<()> {
        let mut c = self.cmd.lock().unwrap();
        c.throttle = output.throttle.clamp(-1.0, 1.0);
        c.steering_rad = output.steering_rad;
        Ok(())
    }

    fn safe_state(&mut self) -> Result<()> {
        let mut c = self.cmd.lock().unwrap();
        c.throttle = 0.0;
        c.steering_rad = 0.0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstar::RTree;

    #[test]
    fn simulated_lidar_uses_positive_y_for_left_side_returns() {
        let world = World {
            inner_walls: RTree::bulk_load(vec![crate::world::Seg {
                ax: -2.0,
                ay: 1.0,
                bx: 2.0,
                by: 1.0,
            }]),
            outer_walls: RTree::new(),
            start: crate::world::StartPose {
                x: 0.0,
                y: 0.0,
                yaw_rad: 0.0,
            },
            waypoints: Vec::new(),
        };
        let (mut source, _actuator) = SimulatorSource::new_with_noise_latency_and_footprint(
            world,
            Box::new(crate::model::KinematicBicycle::default()),
            0.01,
            SensorNoise::zero(),
            LatencyParams::default(),
            VehicleFootprintParams::default(),
        );

        let cloud = source.synthesize_lidar();
        let left_return = cloud
            .points
            .iter()
            .find(|p| p.x.abs() < 0.1 && p.y > 0.9)
            .expect("expected wall return on the left side");
        assert!(left_return.y > 0.0);
    }

    #[test]
    fn footprint_collision_catches_wall_before_center_hits() {
        let world = World {
            inner_walls: RTree::bulk_load(vec![crate::world::Seg {
                ax: -1.0,
                ay: 0.20,
                bx: 1.0,
                by: 0.20,
            }]),
            outer_walls: RTree::new(),
            start: crate::world::StartPose {
                x: 0.0,
                y: 0.0,
                yaw_rad: 0.0,
            },
            waypoints: Vec::new(),
        };
        let (source, _actuator) = SimulatorSource::new_with_noise_latency_and_footprint(
            world,
            Box::new(crate::model::KinematicBicycle::default()),
            0.01,
            SensorNoise::zero(),
            LatencyParams::default(),
            VehicleFootprintParams {
                length_m: 0.40,
                width_m: 0.50,
            },
        );

        assert!(source.is_crashed());
    }

    #[test]
    fn command_latency_delays_model_application() {
        let world = World {
            inner_walls: RTree::new(),
            outer_walls: RTree::new(),
            start: crate::world::StartPose {
                x: 0.0,
                y: 0.0,
                yaw_rad: 0.0,
            },
            waypoints: Vec::new(),
        };
        let (mut source, mut actuator) = SimulatorSource::new_with_noise_latency_and_footprint(
            world,
            Box::new(crate::model::KinematicBicycle::default()),
            0.01,
            SensorNoise::zero(),
            LatencyParams { latency_us: 20_000 },
            VehicleFootprintParams::default(),
        );

        actuator
            .apply(&ControlOutput {
                throttle: 1.0,
                steering_rad: 0.2,
                ..ControlOutput::default()
            })
            .unwrap();

        source.step();
        assert_eq!(source.command().throttle, 0.0);
        source.step();
        assert_eq!(source.command().throttle, 0.0);
        source.step();
        assert_eq!(source.command().throttle, 1.0);
    }
}
