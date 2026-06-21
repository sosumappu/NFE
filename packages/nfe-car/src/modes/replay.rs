use std::sync::Arc;

use anyhow::Result;
use nfe_core::io::SensorSource as CoreSensorSource;
use nfe_runtime::{
    input_replay::McapSensorReplaySource,
    sinks::mcap::McapSink,
    start_gate::{StartGateConfig, StartGateMode},
    telemetry_bus::{TelemetryBus, TelemetrySink},
};
use tracing::{info, warn};

use crate::{
    cli::Args,
    config::Config,
    control::actuate::DryRunActuator,
    control_loop::{self, ControlLoopOptions},
    hal::{ActuatorSink, SensorSource},
    state::SensorSnapshot,
    types::{ImuSample, LidarCloud, LidarPoint},
};

pub async fn run(path: String, fast: bool, args: &Args, config: &Config) -> Result<()> {
    if !fast {
        info!("replay: runtime input replay is deterministic and does not wall-clock pace; ignoring realtime pacing");
    }
    info!("replay: loading {path}");

    let source: Box<dyn SensorSource> = Box::new(RuntimeReplayAdapter::open(&path)?);
    let actuator: Box<dyn ActuatorSink> = Box::new(DryRunActuator);

    let bus = TelemetryBus::new();
    let recorder = if let Some(ref path) = args.record {
        let rx = bus.subscribe(2048);
        match McapSink::start(path, rx) {
            Ok(sink) => {
                info!("replay: re-recording processed telemetry to {path}");
                Some(sink)
            }
            Err(e) => {
                warn!("replay: MCAP sink failed ({e:#}) — continuing without recording");
                None
            }
        }
    } else {
        None
    };

    let mut gate = StartGateConfig::for_mode(StartGateMode::Replay);
    gate.force_arm = args.force_arm;
    gate.replay_start_delay_ms = args
        .replay_start_delay_ms
        .unwrap_or(config.start_gate.replay_start_delay_ms);

    info!("replay: starting pipeline input replay");
    let result = control_loop::run(
        source,
        actuator,
        None,
        Some(bus.clone()),
        config,
        &ControlLoopOptions {
            cost_out: args.cost_out.clone(),
            csv_out: args.csv_out.clone(),
            start_gate_mode: StartGateMode::Replay,
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
        info!("replay: flushing MCAP");
        sink.finish();
    }

    result
}

struct RuntimeReplayAdapter {
    inner: McapSensorReplaySource,
    next: Option<nfe_core::sensors::SensorSnapshot>,
    exhausted: bool,
}

impl RuntimeReplayAdapter {
    fn open(path: &str) -> Result<Self> {
        let mut inner = McapSensorReplaySource::open(path)?;
        let next = inner.next_snapshot()?;
        let exhausted = next.is_none();
        Ok(Self {
            inner,
            next,
            exhausted,
        })
    }
}

impl SensorSource for RuntimeReplayAdapter {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        let Some(current) = self.next.take() else {
            self.exhausted = true;
            anyhow::bail!("replay exhausted");
        };
        self.next = self.inner.next_snapshot()?;
        self.exhausted = self.next.is_none();
        Ok(from_runtime_snapshot(current))
    }

    fn is_exhausted(&self) -> bool {
        self.exhausted
    }
}

fn from_runtime_snapshot(snapshot: nfe_core::sensors::SensorSnapshot) -> SensorSnapshot {
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
    }
}
