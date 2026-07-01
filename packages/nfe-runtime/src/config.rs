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

    #[test]
    fn toml_loads_raceline_solver_params() {
        let path =
            std::env::temp_dir().join(format!("nfe-runtime-raceline-{}.toml", std::process::id()));
        std::fs::write(
            &path,
            "[algo.raceline_solver]\nclearance_m=0.12\nmax_iterations=17\nmax_adjacent_offset_slope=0.04\nvelocity_lateral_accel_limit_ms2=2.5\n",
        )
        .unwrap();

        let config = RuntimeConfig::from_toml_path(&path).unwrap();

        assert_eq!(config.algo.raceline_solver.clearance_m, 0.12);
        assert_eq!(config.algo.raceline_solver.max_iterations, 17);
        assert_eq!(config.algo.raceline_solver.max_adjacent_offset_slope, 0.04);
        assert_eq!(
            config.algo.raceline_solver.velocity_lateral_accel_limit_ms2,
            2.5
        );

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn toml_loads_raceline_controller_params() {
        let path = std::env::temp_dir().join(format!(
            "nfe-runtime-raceline-controller-{}.toml",
            std::process::id()
        ));
        std::fs::write(
            &path,
            "[algo.raceline_controller.lateral]\nnatural_frequency_rad_s=2.4\ndamping_ratio=0.7\nmax_feedback_accel_ms2=9.0\n[algo.raceline_controller.steering]\nwheelbase_m=0.25\nmax_steering_rad=0.6\n[algo.raceline_controller.longitudinal]\nk_speed_ms2_per_ms=5.0\n",
        )
        .unwrap();

        let config = RuntimeConfig::from_toml_path(&path).unwrap();

        assert_eq!(config.algo.raceline_controller.steering.wheelbase_m, 0.25);
        assert_eq!(
            config.algo.raceline_controller.steering.max_steering_rad,
            0.6
        );
        assert_eq!(
            config
                .algo
                .raceline_controller
                .lateral
                .natural_frequency_rad_s,
            2.4
        );
        assert_eq!(config.algo.raceline_controller.lateral.damping_ratio, 0.7);
        assert_eq!(
            config
                .algo
                .raceline_controller
                .lateral
                .max_feedback_accel_ms2,
            9.0
        );
        assert_eq!(
            config
                .algo
                .raceline_controller
                .longitudinal
                .k_speed_ms2_per_ms,
            5.0
        );

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn raceline_solver_accepts_legacy_optimization_iterations_alias() {
        let path = std::env::temp_dir().join(format!(
            "nfe-runtime-raceline-alias-{}.toml",
            std::process::id()
        ));
        std::fs::write(&path, "[algo.raceline_solver]\noptimization_iterations=9\n").unwrap();

        let config = RuntimeConfig::from_toml_path(&path).unwrap();

        assert_eq!(config.algo.raceline_solver.max_iterations, 9);

        let _ = std::fs::remove_file(path);
    }
}
