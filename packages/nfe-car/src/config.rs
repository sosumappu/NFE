use std::path::Path;

use serde::Deserialize;

#[derive(Default, Clone, Debug, Deserialize)]
#[serde(default)]
pub struct Config {
    pub control: ControlConfig,
    pub live: LiveConfig,
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
}

#[derive(Debug, Deserialize, Clone)]
pub struct SpeedConfig {
    pub v_max: f32,
    pub k_dist: f32,
    pub k_heading: f32,
}

#[derive(Debug, Deserialize, Clone)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            watchdog_max_missed: 3,
            lqr: [0.80, 0.30, 1.20, 0.40],
            pid: PidConfig {
                kp: 1.5,
                ki: 0.05,
                kd: 0.2,
            },
            speed: SpeedConfig {
                v_max: 1.0,
                k_dist: 1.0,
                k_heading: 1.0,
            },
            kinematics_horizon: 500,
            hz: 100,
            estop_dist_m: 0.30,
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
    use super::Config;

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
}
