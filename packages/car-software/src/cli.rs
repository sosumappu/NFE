use crate::stream::foxglove_bridge::DEFAULT_PORT;

#[derive(Clone, Debug)]
pub struct Args {
    pub replay: Option<String>,
    pub record: Option<String>,
    pub sim: Option<String>,
    pub model: String,
    pub model_params: Option<String>,
    pub fast: bool,
    pub stream: bool,
    pub stream_port: u16,
    pub cost_out: Option<String>,
    pub csv_out: Option<String>,
    pub config: Option<String>,
}

impl Args {
    pub fn parse() -> Self {
        Self::parse_from(std::env::args())
    }

    pub fn parse_from<I, S>(iter: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        let args: Vec<String> = iter.into_iter().map(Into::into).collect();
        let get = |flag: &str| -> Option<String> {
            args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
        };
        let has = |flag: &str| args.iter().any(|a| a == flag);

        Self {
            replay: get("--replay"),
            record: get("--record"),
            sim: get("--sim"),
            model: get("--model").unwrap_or_else(|| "kinematic".into()),
            model_params: get("--model-params"),
            fast: has("--fast"),
            stream: has("--stream") || std::env::var("STREAM").is_ok(),
            stream_port: get("--stream-port")
                .and_then(|v| v.parse().ok())
                .unwrap_or(DEFAULT_PORT),
            cost_out: get("--cost-out"),
            csv_out: get("--csv-out"),
            config: get("--config"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Args;

    #[test]
    fn parse_reads_known_flags() {
        let args = Args::parse_from([
            "car",
            "--sim",
            "world.json",
            "--model",
            "dynamic",
            "--config",
            "car.toml",
            "--stream-port",
            "9999",
        ]);

        assert_eq!(args.sim.as_deref(), Some("world.json"));
        assert_eq!(args.model, "dynamic");
        assert_eq!(args.config.as_deref(), Some("car.toml"));
        assert_eq!(args.stream_port, 9999);
    }
}
