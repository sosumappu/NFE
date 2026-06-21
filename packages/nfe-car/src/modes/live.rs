use std::sync::Arc;

use anyhow::Result;
#[cfg(target_os = "linux")]
use libsystemd::daemon;
use tracing::{error, info, warn};

use crate::{
    cli::Args,
    config::Config,
    control::actuate::ActuatorFactory,
    control_loop::{self, ControlLoopOptions},
    hal::{ActuatorSink, SensorSource},
    init::{ReadinessBarrier, ReadySignal, Sensor},
    replay::live_source::LiveSensorSource,
    sensors::factory::{SensorFactory, SensorReadySignals},
    state::{SensorStateWriter, SharedState},
};
use nfe_runtime::{
    sinks::mcap::McapSink,
    start_gate::{StartGateConfig, StartGateMode},
    telemetry_bus::{TelemetryBus, TelemetrySink},
};

pub async fn run(args: Args, config: &Config) -> Result<()> {
    let state = SharedState::new();
    let (barrier, _signals) = ReadinessBarrier::new();

    let signals = SensorReadySignals {
        lidar: ReadySignal::dummy(Sensor::Lidar),
        imu: ReadySignal::dummy(Sensor::Imu),
        sonars: vec![
            ReadySignal::dummy(Sensor::Sonar(0)),
            ReadySignal::dummy(Sensor::Sonar(1)),
            ReadySignal::dummy(Sensor::Sonar(2)),
        ],
    };

    // Build the bus before wiring any consumers so subscribe() calls happen
    // before the control loop can call publish(). Subscribers registered after
    // the first publish() would silently miss all earlier events.
    let bus = TelemetryBus::new();

    // MCAP recorder — subscribes first with a large buffer because disk I/O
    // is bursty: a write syscall can stall for milliseconds, and the large
    // buffer absorbs that without dropping frames from the control loop.
    let recorder = if let Some(ref path) = args.record {
        let rx = bus.subscribe(2048);
        match McapSink::start(path, rx) {
            Ok(r) => {
                info!("live: MCAP recording to {path}");
                Some(r)
            }
            Err(e) => {
                warn!("live: MCAP recorder failed ({e:#}) — continuing without recording");
                None
            }
        }
    } else {
        None
    };

    if args.stream {
        warn!(
            "live: --stream is currently unsupported; runtime TelemetryBus recording remains active"
        );
    }

    let state_writer: Arc<dyn SensorStateWriter> = state.clone();
    let spawned = SensorFactory::spawn_all(&state_writer, config.live.lidar_port.clone(), signals);
    if !spawned.skipped.is_empty() {
        warn!("live: degraded sensors: {:?}", spawned.skipped);
    }

    if let Err(e) = barrier.wait_all_ready(config.init.timeout()).await {
        error!("live: INIT FAILED — {e}");
        #[cfg(target_os = "linux")]
        let _ = daemon::notify(
            false,
            &[libsystemd::daemon::NotifyState::Other(
                "STATUS=init failed".into(),
            )],
        );
        std::process::exit(1);
    }

    #[cfg(target_os = "linux")]
    let _ = daemon::notify(false, &[libsystemd::daemon::NotifyState::Ready]);
    info!("live: all sensors ready — starting control loop");

    let source: Box<dyn SensorSource> = Box::new(LiveSensorSource::new(state.clone()));
    let actuator: Box<dyn ActuatorSink> = ActuatorFactory::build(10);

    if args.force_arm && !args.understand_live_force_arm {
        anyhow::bail!("--force-arm in live mode requires --i-understand-live-force-arm");
    }
    let mut gate = StartGateConfig::for_mode(StartGateMode::Live);
    gate.force_arm = args.force_arm;
    gate.allow_live_force_arm = args.understand_live_force_arm;
    let arm_bind = args
        .arm_bind
        .clone()
        .unwrap_or_else(|| config.start_gate.udp_bind.clone());
    let arm_port = args.arm_port.unwrap_or(config.start_gate.udp_port);
    let arm_token = args
        .arm_token
        .clone()
        .unwrap_or_else(|| config.start_gate.udp_token.clone());
    let gpio_enabled = args.gpio_arm || config.start_gate.gpio_enabled;
    let gpio_pin = args.gpio_pin.or(config.start_gate.gpio_pin);

    let result = control_loop::run(
        source,
        actuator,
        Some(state.clone()),
        Some(bus.clone()),
        config,
        &ControlLoopOptions {
            cost_out: args.cost_out.clone(),
            csv_out: args.csv_out.clone(),
            start_gate_mode: StartGateMode::Live,
            start_gate_config: gate,
            arm_udp_bind: Some(format!("{arm_bind}:{arm_port}")),
            arm_udp_token: Some(arm_token),
            arm_gpio_enabled: gpio_enabled,
            arm_gpio_pin: gpio_pin,
        },
    )
    .await;

    info!("live: shutdown — joining sensor threads");
    drop(state_writer);
    for h in spawned.handles {
        if let Err(e) = h.join() {
            warn!("sensor thread panicked: {:?}", e);
        }
    }

    // finish() blocks until the recorder has flushed all buffered events and
    // written the MCAP summary section. Without it the file may be truncated
    // if the process exits before the background thread drains its channel.
    drop(bus);
    if let Some(rec) = recorder {
        info!("live: flushing MCAP");
        rec.finish();
    }

    result
}
