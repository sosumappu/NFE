use std::path::Path;

use nfe_runtime::pipeline::PerceptionMode;
use serde::Deserialize;

#[derive(Default, Clone, Debug, Deserialize)]
#[serde(default)]
pub struct Config {
    pub control: ControlConfig,
    pub live: LiveConfig,
    pub sim: SimConfig,
    pub init: InitConfig,
    pub safety: SafetyConfig,
    pub start_gate: StartGateConfig,
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
    pub watchdog_max_missed: i32,
    pub lqr: [f32; 4],
    pub speed: SpeedConfig,
    pub pid: PidConfig,
    pub stanley: StanleyConfig,
    pub perception: PerceptionConfig,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            watchdog_max_missed: 3,
            lqr: [0.80, 0.30, 1.20, 0.40],
            pid: PidConfig {
                kp: 1.8,
                ki: 0.15,
                kd: 0.4,
                windup_limit: 0.62,
                max_throttle: 1.0,
            },
            perception: PerceptionConfig::default(),
            speed: SpeedConfig {
                v_max: 1.8,
                k_lateral: 1.0,
                k_heading: 5.0,
                obstacle_slowdown_m: 3.0,
            },
            stanley: StanleyConfig::default(),
            kinematics_horizon: 500,
            hz: 100,
            estop_dist_m: 0.30,
        }
    }
}
#[derive(Debug, Deserialize, Clone)]
#[serde(default)]
pub struct PerceptionConfig {
    pub mode: PerceptionMode,
    pub ransac: RansacConfig,
    pub apex: ApexConfig,
}

impl Default for PerceptionConfig {
    fn default() -> Self {
        Self {
            mode: PerceptionMode::Corridor,
            ransac: RansacConfig::default(),
            apex: ApexConfig::default(),
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(default)]
pub struct RansacConfig {
    pub inlier_dist_m: f32,
    pub min_inliers: usize,
    pub iterations: usize,
    pub max_walls: usize,
    pub min_pair_sep_m: f32,
}

impl Default for RansacConfig {
    fn default() -> Self {
        Self {
            inlier_dist_m: 0.2,
            min_inliers: 9,
            iterations: 80,
            max_walls: 4,
            min_pair_sep_m: 0.02,
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(default)]
pub struct ApexConfig {
    pub median_window: usize,
    pub min_points: usize,
    pub min_forward_m: f32,
    pub min_range_jump_m: f32,
    pub max_opposite_dist_error_m: f32,
    pub max_lookahead_m: f32,
    pub min_lookahead_m: f32,
    pub lookahead_sensitivity: f32,
    pub side_lookahead_fov_deg: f32,
    pub side_lookahead_center_deg: f32,
}

impl Default for ApexConfig {
    fn default() -> Self {
        Self {
            median_window: 5,
            min_points: 8,
            min_forward_m: 0.05,
            min_range_jump_m: 0.08,
            max_opposite_dist_error_m: 0.75,
            max_lookahead_m: 8.0,
            min_lookahead_m: 0.5,
            lookahead_sensitivity: 5.0,
            side_lookahead_fov_deg: 80.0,
            side_lookahead_center_deg: 90.0,
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub windup_limit: f32,
    pub max_throttle: f32,
}

#[derive(Debug, Deserialize, Clone)]
pub struct SpeedConfig {
    pub v_max: f32,
    pub k_lateral: f32,
    pub k_heading: f32,
    pub obstacle_slowdown_m: f32,
}
#[derive(Debug, Deserialize, Clone)]
#[serde(default)]
pub struct StanleyConfig {
    pub k_cross_track: f32,
    pub softening_speed_ms: f32,
    pub max_steering_rad: f32,
}

impl Default for StanleyConfig {
    fn default() -> Self {
        Self {
            k_cross_track: 1.0,
            softening_speed_ms: 1.0,
            max_steering_rad: 0.38,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct StartGateConfig {
    pub udp_bind: String,
    pub udp_port: u16,
    pub udp_token: String,
    pub gpio_enabled: bool,
    pub gpio_pin: Option<u8>,
    pub sim_start_delay_ms: u64,
    pub replay_start_delay_ms: u64,
}

impl Default for StartGateConfig {
    fn default() -> Self {
        Self {
            udp_bind: "0.0.0.0".to_string(),
            udp_port: 4578,
            udp_token: "nfe".to_string(),
            gpio_enabled: false,
            gpio_pin: None,
            sim_start_delay_ms: 100,
            replay_start_delay_ms: 0,
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
pub struct SimConfig {
    #[serde(flatten)]
    pub footprint: nfe_sim::VehicleFootprintParams,
    pub kinematic: nfe_sim::KinematicBicycleParams,
    pub dynamic: nfe_sim::DynamicBicycleParams,
    pub latency: nfe_sim::LatencyParams,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            footprint: nfe_sim::VehicleFootprintParams::default(),
            kinematic: nfe_sim::KinematicBicycleParams::default(),
            dynamic: nfe_sim::DynamicBicycleParams::default(),
            latency: nfe_sim::LatencyParams::default(),
        }
    }
}

// ── Safety configuration (tunable constants moved out of code) ───────────-
#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct SafetyConfig {
    pub wheelbase_m: f32,
    pub half_channel_w_m: f32,
    pub t_lookahead_s: f32,
    pub estop_min_m: f32,
    pub estop_max_m: f32,
    pub c_arc: f32,
    pub tan_min: f32,
    pub rearm_gap_m: f32,
    pub n_clean_ticks: u32,
    pub min_front_points: u32,
    pub lidar_stale_ms: u64,
    pub blind_grace_ms: u64,
    pub imu_stale_ms: u64,
    pub sonar_stale_ms: u64,
    pub escalate_at: u32,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            wheelbase_m: 0.215,
            half_channel_w_m: 0.13,
            t_lookahead_s: 0.4,
            estop_min_m: 0.25,
            estop_max_m: 1.5,
            c_arc: 0.5,
            tan_min: 0.0717,
            rearm_gap_m: 0.20,
            n_clean_ticks: 5,
            min_front_points: 4,
            lidar_stale_ms: 350,
            blind_grace_ms: 350,
            imu_stale_ms: 20,
            sonar_stale_ms: 100,
            escalate_at: 6,
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

#[cfg(test)]
mod tests {
    use super::{Config, PerceptionMode};

    #[test]
    fn toml_loads_and_defaults_missing_fields() {
        let path = std::env::temp_dir().join(format!("nfe-{}.toml", std::process::id()));
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

    #[test]
    fn toml_loads_sim_config() {
        let path = std::env::temp_dir().join(format!("nfe-sim-{}.toml", std::process::id()));
        std::fs::write(
            &path,
            "[sim]\nlength_m=0.44\nwidth_m=0.27\n[sim.dynamic]\nmass_kg=2.0\n[sim.dynamic.servo]\ntau_s=0.07\n[sim.dynamic.drivetrain]\nfront_drive_fraction=0.6\n[sim.dynamic.low_speed]\nblend_end_ms=0.4\n[sim.latency]\nlatency_us=30000\n",
        )
        .unwrap();

        let config = Config::from_toml_path(&path).unwrap();
        assert_eq!(config.sim.footprint.length_m, 0.44);
        assert_eq!(config.sim.footprint.width_m, 0.27);
        assert_eq!(config.sim.dynamic.mass, 2.0);
        assert_eq!(config.sim.dynamic.servo.tau_s, 0.07);
        assert_eq!(config.sim.dynamic.drivetrain.front_drive_fraction, 0.6);
        assert_eq!(config.sim.dynamic.low_speed.blend_end_ms, 0.4);
        assert_eq!(config.sim.latency.latency_us, 30_000);
        assert_eq!(config.sim.kinematic.wheelbase, 0.33);

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn toml_loads_partial_stanley_and_ransac_params() {
        let path = std::env::temp_dir().join(format!("nfe-params-{}.toml", std::process::id()));
        std::fs::write(
            &path,
            "[control.stanley]\nk_cross_track=3.25\n[control.perception]\nmode='apex'\n[control.perception.ransac]\niterations=123\nmax_walls=6\n[control.perception.apex]\nmedian_window=5\n",
        )
        .unwrap();

        let config = Config::from_toml_path(&path).unwrap();
        assert_eq!(config.control.stanley.k_cross_track, 3.25);
        assert_eq!(config.control.stanley.softening_speed_ms, 1.0);
        assert_eq!(config.control.stanley.max_steering_rad, 0.38);
        assert_eq!(config.control.perception.mode, PerceptionMode::Apex);
        assert_eq!(config.control.perception.ransac.iterations, 123);
        assert_eq!(config.control.perception.ransac.max_walls, 6);
        assert_eq!(config.control.perception.ransac.inlier_dist_m, 0.2);
        assert_eq!(config.control.perception.apex.median_window, 5);
        assert_eq!(config.control.perception.apex.min_points, 8);

        let _ = std::fs::remove_file(path);
    }
}
