const DEFAULT_STREAM_PORT: u16 = 8765;

#[derive(Clone, Debug)]
pub struct Args {
    pub replay: Option<String>,
    pub record: Option<String>,
    pub sim: Option<String>,
    pub model: String,
    pub model_params: Option<String>,
    pub sim_seed: Option<u64>,
    pub fast: bool,
    pub stream: bool,
    pub stream_port: u16,
    pub cost_out: Option<String>,
    pub csv_out: Option<String>,
    pub config: Option<String>,
    pub force_arm: bool,
    pub understand_live_force_arm: bool,
    pub sim_start_delay_ms: Option<u64>,
    pub replay_start_delay_ms: Option<u64>,
    pub arm_bind: Option<String>,
    pub arm_port: Option<u16>,
    pub arm_token: Option<String>,
    pub gpio_arm: bool,
    pub gpio_pin: Option<u8>,
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
            sim_seed: get("--sim-seed").and_then(|v| v.parse().ok()),
            fast: has("--fast"),
            stream: has("--stream") || std::env::var("STREAM").is_ok(),
            stream_port: get("--stream-port")
                .and_then(|v| v.parse().ok())
                .unwrap_or(DEFAULT_STREAM_PORT),
            cost_out: get("--cost-out"),
            csv_out: get("--csv-out"),
            config: get("--config"),
            force_arm: has("--force-arm"),
            understand_live_force_arm: has("--i-understand-live-force-arm"),
            sim_start_delay_ms: get("--sim-start-delay-ms").and_then(|v| v.parse().ok()),
            replay_start_delay_ms: get("--replay-start-delay-ms").and_then(|v| v.parse().ok()),
            arm_bind: get("--arm-bind"),
            arm_port: get("--arm-port").and_then(|v| v.parse().ok()),
            arm_token: get("--arm-token"),
            gpio_arm: has("--gpio-arm"),
            gpio_pin: get("--gpio-pin").and_then(|v| v.parse().ok()),
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
            "--sim-seed",
            "42",
            "--stream-port",
            "9999",
            "--force-arm",
            "--sim-start-delay-ms",
            "0",
            "--gpio-arm",
            "--gpio-pin",
            "17",
        ]);

        assert_eq!(args.sim.as_deref(), Some("world.json"));
        assert_eq!(args.model, "dynamic");
        assert_eq!(args.config.as_deref(), Some("car.toml"));
        assert_eq!(args.sim_seed, Some(42));
        assert_eq!(args.stream_port, 9999);
        assert!(args.force_arm);
        assert_eq!(args.sim_start_delay_ms, Some(0));
        assert!(args.gpio_arm);
        assert_eq!(args.gpio_pin, Some(17));
    }
}
