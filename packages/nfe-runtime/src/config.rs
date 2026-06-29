use std::path::Path;

use nfe_algo::config::AlgoConfig;

use crate::pipeline::PerceptionMode;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct RuntimeConfig {
    pub hz: u64,
    pub perception_mode: PerceptionMode,
    pub mapping: MappingRuntimeConfig,
    pub algo: AlgoConfig,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            hz: 100,
            perception_mode: PerceptionMode::Apex,
            mapping: MappingRuntimeConfig::default(),
            algo: AlgoConfig::default(),
        }
    }
}

impl RuntimeConfig {
    pub fn from_toml_path(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let body = std::fs::read_to_string(path)?;
        Ok(toml::from_str(&body)?)
    }

    pub fn dt_s(&self) -> f32 {
        1.0 / self.hz as f32
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct MappingRuntimeConfig {
    /// Single flag that disables mapping entirely, independent of DriveMode.
    pub enabled: bool,
    /// Bounded worker queue capacity; full queues drop scans rather than
    /// blocking the control loop.
    pub queue_capacity: usize,
}

impl Default for MappingRuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            queue_capacity: 4,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::RuntimeConfig;

    #[test]
    fn toml_loads_apex_runtime_params() {
        let path =
            std::env::temp_dir().join(format!("nfe-runtime-apex-{}.toml", std::process::id()));
        std::fs::write(
            &path,
            "[algo.apex]\nprefer_nearer_opposite=false\nwall_clearance_m=0.22\napex_switch_threshold_rad=0.45\napex_switch_hysteresis_factor=2.2\n",
        )
        .unwrap();

        let config = RuntimeConfig::from_toml_path(&path).unwrap();

        assert!(!config.algo.apex.prefer_nearer_opposite);
        assert_eq!(config.algo.apex.wall_clearance_m, 0.22);
        assert_eq!(config.algo.apex.apex_switch_threshold_rad, 0.45);
        assert_eq!(config.algo.apex.apex_switch_hysteresis_factor, 2.2);

        let _ = std::fs::remove_file(path);
    }
}
