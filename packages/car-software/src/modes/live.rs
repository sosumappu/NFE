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
    replay::{live_source::LiveSensorSource, recorder::McapRecorder},
    sensors::factory::{SensorFactory, SensorReadySignals},
    state::{SensorStateWriter, SharedState},
    stream::foxglove_bridge::FoxgloveBridge,
    telemetry::TelemetryBus,
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
        match McapRecorder::start(path, rx) {
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

    // Foxglove bridge — subscribes after the recorder so its smaller buffer
    // does not starve the recorder's channel during startup. The bridge polls
    // every 50 ms and only needs the latest metrics, so 64 events is generous.
    let _bridge = if args.stream {
        match FoxgloveBridge::start(state.clone(), &bus, args.stream_port, 50).await {
            Ok(b) => {
                info!("live: Foxglove bridge on ws://0.0.0.0:{}", args.stream_port);
                Some(b)
            }
            Err(e) => {
                warn!("live: Foxglove bridge failed ({e:#})");
                None
            }
        }
    } else {
        None
    };

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

    let result = control_loop::run(
        source,
        actuator,
        Some(state.clone()),
        Some(bus),
        config,
        &ControlLoopOptions {
            cost_out: args.cost_out.clone(),
            csv_out: args.csv_out.clone(),
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
    if let Some(rec) = recorder {
        info!("live: flushing MCAP");
        rec.finish();
    }

    result
}
