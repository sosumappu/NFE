use std::path::Path;

use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct Config {
    pub control: ControlConfig,
    pub live: LiveConfig,
    pub init: InitConfig,
    pub observability: ObservabilityConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            control: ControlConfig::default(),
            live: LiveConfig::default(),
            init: InitConfig::default(),
            observability: ObservabilityConfig::default(),
        }
    }
}

impl Config {
    pub fn load(path: Option<&str>) -> Self {
        match path {
            Some(p) => Self::from_toml_path(p).unwrap_or_else(|e| {
                eprintln!("config: failed to load {p}: {e:#}; using defaults");
                Self::default()
            }),
            None => Self::default(),
        }
    }

    pub fn from_toml_path(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let body = std::fs::read_to_string(path.as_ref())?;
        Ok(toml::from_str(&body)?)
    }

    pub fn control_period(&self) -> std::time::Duration {
        std::time::Duration::from_millis(1000 / self.control.hz)
    }

    pub fn control_dt(&self) -> f32 {
        1.0 / self.control.hz as f32
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct ControlConfig {
    pub kinematics_horizon: usize,
    pub hz: u64,
    pub estop_dist_m: f32,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            kinematics_horizon: 500,
            hz: 100,
            estop_dist_m: 0.30,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct LiveConfig {
    pub lidar_port: String,
}

impl Default for LiveConfig {
    fn default() -> Self {
        Self {
            lidar_port: "/dev/lidar".to_string(),
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct InitConfig {
    pub timeout_secs: u64,
}

impl Default for InitConfig {
    fn default() -> Self {
        Self { timeout_secs: 5 }
    }
}

impl InitConfig {
    pub fn timeout(&self) -> std::time::Duration {
        std::time::Duration::from_secs(self.timeout_secs)
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct ObservabilityConfig {
    pub prometheus_bind: Option<String>,
}

impl Default for ObservabilityConfig {
    fn default() -> Self {
        Self {
            prometheus_bind: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Config;

    #[test]
    fn toml_loads_and_defaults_missing_fields() {
        let path = std::env::temp_dir().join(format!("nfe-config-{}.toml", std::process::id()));
        std::fs::write(&path, "[control]\nhz=50\n[live]\nlidar_port='/tmp/lidar'\n").unwrap();

        let config = Config::from_toml_path(&path).unwrap();
        assert_eq!(config.control.hz, 50);
        assert_eq!(config.control.estop_dist_m, 0.30);
        assert_eq!(config.live.lidar_port, "/tmp/lidar");

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn load_falls_back_to_defaults() {
        let config = Config::load(Some("/no/such/file.toml"));
        assert_eq!(config.control.hz, 100);
        assert_eq!(config.live.lidar_port, "/dev/lidar");
    }
}
