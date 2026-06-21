use anyhow::Result;
use tokio::runtime::Builder;
use tracing::info;

use nfe_car::{bootstrap, cli::Args, modes, observability};

fn main() -> Result<()> {
    observability::init_tracing()?;
    let args = Args::parse();
    info!("car: NFE starting");

    let config = bootstrap::initialize(&args);

    let rt = Builder::new_current_thread()
        .enable_time()
        .enable_io()
        .build()?;

    rt.block_on(async move { modes::dispatch(args, config).await })
}
