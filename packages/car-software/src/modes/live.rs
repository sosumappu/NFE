use std::sync::Arc;

use anyhow::Result;
use libsystemd::daemon::{self, NotifyState};
use tracing::{error, info, warn};

use crate::{
    cli::Args,
    config::Config,
    control::actuate::ActuatorFactory,
    control_loop::{self, ControlLoopOptions},
    hal::{ActuatorSink, SensorSource},
    init::{ReadinessBarrier, ReadySignal, Sensor},
    observability::Observability,
    replay::{live_source::LiveSensorSource, recorder::McapRecorder},
    sensors::factory::{SensorFactory, SensorReadySignals},
    state::{SensorStateWriter, SharedState},
    stream::foxglove_bridge::FoxgloveBridge,
};

pub async fn run(args: Args, config: &Config, observability: &Observability) -> Result<()> {
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

    let (recorder, mcap_tx): (Option<McapRecorder>, Option<_>) = if let Some(ref path) = args.record
    {
        match McapRecorder::start(path) {
            Ok(r) => {
                let tx = r.sender();
                info!("live: MCAP recording to {path}");
                (Some(r), Some(tx))
            }
            Err(e) => {
                warn!("live: MCAP recorder failed ({e:#}) — continuing without recording");
                (None, None)
            }
        }
    } else {
        (None, None)
    };

    let state_writer: Arc<dyn SensorStateWriter> = state.clone();
    let spawned = SensorFactory::spawn_all(&state_writer, config.live.lidar_port.clone(), signals);
    if !spawned.skipped.is_empty() {
        warn!("live: degraded sensors: {:?}", spawned.skipped);
    }

    let _bridge = if args.stream {
        match FoxgloveBridge::start(state.clone(), args.stream_port, 50) {
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

    if let Err(e) = barrier.wait_all_ready(config.init.timeout()).await {
        error!("live: INIT FAILED — {e}");
        let _ = daemon::notify(false, &[NotifyState::Other("STATUS=init failed".into())]);
        std::process::exit(1);
    }

    let _ = daemon::notify(false, &[NotifyState::Ready]);
    info!("live: all sensors ready — starting control loop");

    let source: Box<dyn SensorSource> = Box::new(LiveSensorSource::new(state.clone()));
    let actuator: Box<dyn ActuatorSink> = ActuatorFactory::build(10);

    let result = control_loop::run(
        source,
        actuator,
        Some(state.clone()),
        mcap_tx,
        config,
        observability,
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
    if let Some(rec) = recorder {
        info!("live: flushing MCAP");
        rec.finish();
    }

    result
}
