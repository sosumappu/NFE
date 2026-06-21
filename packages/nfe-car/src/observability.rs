use anyhow::Result;

pub fn init_tracing() -> Result<()> {
    use tracing_subscriber::prelude::*;
    let fmt = tracing_subscriber::fmt::layer().with_filter(
        tracing_subscriber::EnvFilter::from_env("RUST_LOG")
            .add_directive("car=debug".parse().unwrap())
            .add_directive("nfe_car=warn".parse().unwrap())
            .add_directive("nfe_car::control_loop=error".parse().unwrap())
            .add_directive("nfe_car::modes=info".parse().unwrap())
            .add_directive("nfe_runtime=info".parse().unwrap())
            .add_directive("nfe_sim=warn".parse().unwrap()),
    );
    #[cfg(target_os = "linux")]
    if std::env::var("JOURNAL_STREAM").is_ok() {
        let jd = tracing_journald::layer()?;
        tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt).with(jd))?;
        return Ok(());
    }

    tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt))?;
    Ok(())
}
