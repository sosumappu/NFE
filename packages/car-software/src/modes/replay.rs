use anyhow::Result;
use tracing::info;

use crate::{
    cli::Args,
    config::Config,
    control::actuate::ActuatorFactory,
    control_loop::{self, ControlLoopOptions},
    hal::{ActuatorSink, SensorSource},
    replay::replayer::{McapReplayer, ReplayMode},
};

pub async fn run(path: String, fast: bool, args: &Args, config: &Config) -> Result<()> {
    info!("replay: loading {path}");
    let mode = if fast {
        ReplayMode::Fast
    } else {
        ReplayMode::Realtime
    };

    let source: Box<dyn SensorSource> = Box::new(McapReplayer::open(&path, mode)?);
    let actuator: Box<dyn ActuatorSink> = ActuatorFactory::build(1);

    info!("replay: starting control loop ({mode:?})");

    // No bus in replay: there are no live consumers (no bridge, no recorder)
    // and the metrics are only needed for the post-run cost summary which
    // control_loop writes to its internal MetricsLog regardless.
    control_loop::run(
        source,
        actuator,
        None,
        None,
        config,
        &ControlLoopOptions {
            cost_out: args.cost_out.clone(),
            csv_out: args.csv_out.clone(),
        },
    )
    .await
}
