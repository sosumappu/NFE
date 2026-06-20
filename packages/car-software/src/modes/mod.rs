use anyhow::Result;

use crate::{cli::Args, config::Config};

pub mod live;
pub mod replay;
pub mod sim;

pub async fn dispatch(args: Args, config: Config) -> Result<()> {
    match (&args.replay, &args.sim) {
        (Some(path), _) => replay::run(path.clone(), args.fast, &args, &config).await,
        (_, Some(path)) => sim::run(path.clone(), &args, &config).await,
        _ => live::run(args, &config).await,
    }
}
