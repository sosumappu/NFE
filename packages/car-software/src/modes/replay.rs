use anyhow::Result;
use tracing::info;

use crate::{
    cli::Args,
    config::Config,
    control::actuate::ActuatorFactory,
    control_loop::{self, ControlLoopOptions},
    hal::{ActuatorSink, SensorSource},
    observability::Observability,
    replay::replayer::{McapReplayer, ReplayMode},
};

pub async fn run(
    path: String,
    fast: bool,
    args: &Args,
    config: &Config,
    observability: &Observability,
) -> Result<()> {
    info!("replay: loading {path}");
    let mode = if fast {
        ReplayMode::Fast
    } else {
        ReplayMode::Realtime
    };
    let source: Box<dyn SensorSource> = Box::new(McapReplayer::open(&path, mode)?);
    let actuator: Box<dyn ActuatorSink> = ActuatorFactory::build(1);
    info!("replay: starting control loop ({mode:?})");
    control_loop::run(
        source,
        actuator,
        None,
        None,
        config,
        observability,
        &ControlLoopOptions {
            cost_out: args.cost_out.clone(),
            csv_out: args.csv_out.clone(),
        },
    )
    .await
}
