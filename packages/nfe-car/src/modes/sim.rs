use std::sync::Arc;

use anyhow::Result;
use nfe_core::io::{ActuatorSink as CoreActuatorSink, SensorSource as CoreSensorSource};
use nfe_runtime::{
    session,
    sinks::mcap::McapSink,
    start_gate::{StartGateConfig, StartGateMode},
    telemetry_bus::{TelemetryBus, TelemetrySink},
};
use nfe_sim::{
    telemetry::{ground_truth_event_with_footprint, world_snapshot_event},
    DynamicBicycle, IdentifiedModel, KinematicBicycle, SimActuator, SimulatorSource, VehicleModel,
    World,
};
use tracing::{info, warn};

use crate::{
    cli::Args,
    config::Config,
    control_loop::{self, ControlLoopOptions},
    hal::{ActuatorSink, SensorSource},
    state::SensorSnapshot,
    types::{ImuSample, LidarCloud, LidarPoint},
};

pub async fn run(world_path: String, args: &Args, config: &Config) -> Result<()> {
    info!("sim: loading world from {world_path}");
    let world = World::load(&world_path)?;
    info!(
        "sim: {} inner_walls {} outer_walls  {} waypoints  model={}",
        world.inner_walls.size(),
        world.outer_walls.size(),
        world.waypoints.len(),
        args.model
    );

    let model: Box<dyn VehicleModel> = match args.model.as_str() {
        "dynamic" => Box::new(DynamicBicycle::from_params(config.sim.dynamic)),
        "identified" => {
            let p = args
                .model_params
                .as_deref()
                .ok_or_else(|| anyhow::anyhow!("--model-params required"))?;
            Box::new(IdentifiedModel::from_json_with_base(p, config.sim.dynamic)?)
        }
        _ => Box::new(KinematicBicycle::from_params(config.sim.kinematic)),
    };

    let (source, actuator) = if let Some(seed) = args.sim_seed {
        info!(seed, "sim: deterministic noise seed");
        SimulatorSource::new_with_seed_latency_and_footprint(
            world,
            model,
            config.control_dt(),
            seed,
            config.sim.latency,
            config.sim.footprint,
        )
    } else {
        SimulatorSource::new_with_latency_and_footprint(
            world,
            model,
            config.control_dt(),
            config.sim.latency,
            config.sim.footprint,
        )
    };

    let bus = TelemetryBus::new();

    let recorder = if let Some(ref path) = args.record {
        let rx = bus.subscribe(2048);
        match McapSink::start(path, rx) {
            Ok(sink) => {
                info!("sim: MCAP recording to {path}");
                Some(sink)
            }
            Err(e) => {
                warn!("sim: MCAP sink failed ({e:#}) — continuing without recording");
                None
            }
        }
    } else {
        None
    };

    session::publish_event(&bus, world_snapshot_event(source.world(), 0));

    let mut gate = StartGateConfig::for_mode(StartGateMode::Sim);
    gate.force_arm = args.force_arm;
    gate.sim_start_delay_ms = args
        .sim_start_delay_ms
        .unwrap_or(config.start_gate.sim_start_delay_ms);

    let result = control_loop::run(
        Box::new(SimSourceAdapter::new(source)),
        Box::new(SimActuatorAdapter {
            inner: actuator,
            command: nfe_core::control::ControlOutput::default(),
        }),
        None,
        Some(bus.clone()),
        config,
        &ControlLoopOptions {
            cost_out: args.cost_out.clone(),
            csv_out: args.csv_out.clone(),
            start_gate_mode: StartGateMode::Sim,
            start_gate_config: gate,
            arm_udp_bind: None,
            arm_udp_token: None,
            arm_gpio_enabled: false,
            arm_gpio_pin: None,
        },
    )
    .await;

    drop(bus);
    if let Some(sink) = recorder {
        info!("sim: flushing MCAP");
        sink.finish();
    }

    result
}

struct SimSourceAdapter {
    inner: SimulatorSource,
    exhausted: bool,
    pending_events: Vec<nfe_core::telemetry::TelemetryEvent>,
}

impl SimSourceAdapter {
    fn new(inner: SimulatorSource) -> Self {
        Self {
            inner,
            exhausted: false,
            pending_events: Vec::new(),
        }
    }
}

impl SensorSource for SimSourceAdapter {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        let Some(snapshot) = self.inner.next_snapshot()? else {
            self.exhausted = true;
            let reason = self.inner.exhaustion_reason().unwrap_or("sim exhausted");
            anyhow::bail!("{reason}");
        };
        self.pending_events.push(ground_truth_event_with_footprint(
            self.inner.vehicle_state(),
            self.inner.command(),
            self.inner.timestamp_us(),
            self.inner.footprint(),
        ));
        Ok(from_core_snapshot(snapshot))
    }

    fn is_exhausted(&self) -> bool {
        self.exhausted || self.inner.is_exhausted()
    }

    fn telemetry_events(&mut self) -> Vec<nfe_core::telemetry::TelemetryEvent> {
        std::mem::take(&mut self.pending_events)
    }
}

struct SimActuatorAdapter {
    inner: SimActuator,
    command: nfe_core::control::ControlOutput,
}

impl ActuatorSink for SimActuatorAdapter {
    fn set_throttle(&mut self, throttle: f32) -> Result<()> {
        self.command.throttle = throttle;
        self.inner.apply(&self.command)
    }

    fn set_steering(&mut self, angle_rad: f32) -> Result<()> {
        self.command.steering_rad = angle_rad;
        self.inner.apply(&self.command)
    }

    fn safe_state(&mut self) -> Result<()> {
        self.command = nfe_core::control::ControlOutput::default();
        self.inner.safe_state()
    }

    fn label(&self) -> &'static str {
        "sim"
    }
}

fn from_core_snapshot(snapshot: nfe_core::sensors::SensorSnapshot) -> SensorSnapshot {
    SensorSnapshot {
        lidar: Arc::new(LidarCloud {
            timestamp_us: snapshot.lidar.timestamp_us,
            points: snapshot
                .lidar
                .points
                .into_iter()
                .map(|p| LidarPoint {
                    x: p.x,
                    y: p.y,
                    dist_m: p.dist_m,
                    angle_rad: p.angle_rad,
                    timestamp_us: p.timestamp_us,
                })
                .collect(),
        }),
        imu: ImuSample {
            ax: snapshot.imu.ax,
            ay: snapshot.imu.ay,
            az: snapshot.imu.az,
            gx: snapshot.imu.gx,
            gy: snapshot.imu.gy,
            gz: snapshot.imu.gz,
            timestamp_us: snapshot.imu.timestamp_us,
        },
        sonar_m: snapshot.sonar_m,
        sensor_fault: snapshot.sensor_fault,
        start_line_crossed: snapshot.start_line_crossed,
    }
}
