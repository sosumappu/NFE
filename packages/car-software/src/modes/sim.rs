use anyhow::Result;
use tracing::info;

use crate::{
    cli::Args,
    config::Config,
    control_loop::{self, ControlLoopOptions},
    observability::Observability,
    sim::{
        model::{DynamicBicycle, IdentifiedModel, KinematicBicycle, VehicleModel},
        source::SimulatorSource,
        world::World,
    },
};

pub async fn run(
    world_path: String,
    args: &Args,
    config: &Config,
    observability: &Observability,
) -> Result<()> {
    info!("sim: loading world from {world_path}");
    let world = World::load(&world_path)?;
    info!(
        "sim: {} walls  {} waypoints  model={}",
        world.walls.len(),
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
    control_loop::run(
        Box::new(source),
        Box::new(actuator),
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
