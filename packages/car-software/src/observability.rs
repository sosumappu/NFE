use anyhow::Result;

use crate::config::Config;

#[derive(Clone, Debug, Default)]
pub struct Observability;

impl Observability {
    pub fn setup(config: &Config) -> Result<Self> {
        #[cfg(feature = "prometheus")]
        {
            if let Some(bind) = &config.observability.prometheus_bind {
                tracing::info!("prometheus: enabled on {bind}");
            }
        }

        #[cfg(not(feature = "prometheus"))]
        if config.observability.prometheus_bind.is_some() {
            tracing::warn!(
                "prometheus: requested in config but feature disabled; rebuilding with --features prometheus is required"
            );
        }

        Ok(Self)
    }
}

pub fn init_tracing() -> Result<()> {
    use tracing_subscriber::prelude::*;
    let fmt = tracing_subscriber::fmt::layer().with_filter(
        tracing_subscriber::EnvFilter::from_env("RUST_LOG")
            .add_directive("car=debug".parse().unwrap()),
    );
    if std::env::var("JOURNAL_STREAM").is_ok() {
        let jd = tracing_journald::layer()?;
        tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt).with(jd))?;
    } else {
        tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt))?;
    }
    Ok(())
}
