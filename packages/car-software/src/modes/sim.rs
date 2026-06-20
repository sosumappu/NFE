use anyhow::Result;
use tracing::info;

use crate::{
    cli::Args,
    config::Config,
    control_loop::{self, ControlLoopOptions},
    sim::{
        model::{DynamicBicycle, IdentifiedModel, KinematicBicycle, VehicleModel},
        source::SimulatorSource,
        world::World,
    },
};

pub async fn run(world_path: String, args: &Args, config: &Config) -> Result<()> {
    info!("sim: loading world from {world_path}");
    let world = World::load(&world_path)?;
    info!(
        "sim: {} inner_walls {} outer_walls  {} waypoints  model={}",
        world.inner_walls.len(),
        world.outer_walls.len(),
        world.waypoints.len(),
        args.model
    );

    let model: Box<dyn VehicleModel> = match args.model.as_str() {
        "dynamic" => Box::new(DynamicBicycle::default()),
        "identified" => {
            let p = args
                .model_params
                .as_deref()
                .ok_or_else(|| anyhow::anyhow!("--model-params required"))?;
            Box::new(IdentifiedModel::from_json(p)?)
        }
        _ => Box::new(KinematicBicycle::default()),
    };

    let (source, actuator) = SimulatorSource::new(world, model, config.control_dt());

    // No bus in sim: the tuner (bin/tune.rs) calls run_episode() directly and
    // reads cost from MetricsLog::summarise(). Wiring a bus here would require
    // a subscriber thread that immediately drops events, adding overhead to the
    // tight CMA-ES evaluation loop for no benefit.
    control_loop::run(
        Box::new(source),
        Box::new(actuator),
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
