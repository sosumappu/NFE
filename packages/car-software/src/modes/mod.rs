use anyhow::Result;

use crate::{cli::Args, config::Config, observability::Observability};

pub mod live;
pub mod replay;
pub mod sim;

pub async fn dispatch(args: Args, config: Config, observability: Observability) -> Result<()> {
    match (&args.replay, &args.sim) {
        (Some(path), _) => {
            replay::run(path.clone(), args.fast, &args, &config, &observability).await
        }
        (_, Some(path)) => sim::run(path.clone(), &args, &config, &observability).await,
        _ => live::run(args, &config, &observability).await,
    }
}
