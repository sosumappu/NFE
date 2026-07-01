use std::collections::HashMap;
use std::path::Path;

use anyhow::{bail, Context, Result};
use nfe_core::params::ParamSpec;
use nfe_runtime::config::{MappingRuntimeConfig, RuntimeConfig};
use nfe_runtime::pipeline::PerceptionMode;
use serde::Serialize;
use serde_json::{Number, Value};

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize, PartialEq)]
pub struct Candidate {
    pub params: HashMap<String, f64>,
}

impl Candidate {
    pub fn apply_to_runtime_config(&self, base: &RuntimeConfig) -> Result<RuntimeConfig> {
        let specs: HashMap<_, _> = nfe_runtime::tuning::search_space().into_iter().collect();
        let mut value = serde_json::to_value(base)?;

        for (key, raw) in &self.params {
            let spec = specs
                .get(key)
                .with_context(|| format!("unknown tunable parameter: {key}"))?;
            if !raw.is_finite() {
                bail!("candidate parameter {key} is not finite: {raw}");
            }
            set_json_path(&mut value, key, param_value(*spec, *raw)?)?;
        }

        Ok(serde_json::from_value(value)?)
    }
}

pub fn load_runtime_config(path: impl AsRef<Path>) -> Result<RuntimeConfig> {
    let body = std::fs::read_to_string(path.as_ref())?;
    let value: toml::Value = toml::from_str(&body)?;
    if value.get("control").is_some() {
        let car: CarConfigCompat = value.try_into()?;
        Ok(runtime_config_from_car_compat(&car))
    } else {
        Ok(toml::from_str(&body)?)
    }
}

pub fn runtime_config_from_car_config(config: &impl Serialize) -> Result<RuntimeConfig> {
    let value = serde_json::to_value(config)?;
    let car: CarConfigCompat = serde_json::from_value(value)?;
    Ok(runtime_config_from_car_compat(&car))
}

pub fn validate_runtime_config(config: &RuntimeConfig) -> Vec<String> {
    let mut errors = Vec::new();
    let apex = &config.algo.apex;
    if !apex.min_lookahead_m.is_finite() || !apex.max_lookahead_m.is_finite() {
        errors.push("algo.apex lookahead bounds must be finite".to_string());
    } else if apex.min_lookahead_m > apex.max_lookahead_m {
        errors.push(format!(
            "algo.apex.min_lookahead_m ({}) must be <= algo.apex.max_lookahead_m ({})",
            apex.min_lookahead_m, apex.max_lookahead_m
        ));
    }
    errors
}

fn param_value(spec: ParamSpec, raw: f64) -> Result<Value> {
    let clamped = spec.clamp(raw);
    match spec {
        ParamSpec::Continuous { .. } => Number::from_f64(clamped)
            .map(Value::Number)
            .context("continuous parameter did not produce a finite JSON number"),
        ParamSpec::Integer { .. } => Ok(Value::Number(Number::from(clamped as i64))),
    }
}

fn set_json_path(root: &mut Value, key: &str, replacement: Value) -> Result<()> {
    let mut current = root;
    let mut parts = key.split('.').peekable();
    while let Some(part) = parts.next() {
        let object = current
            .as_object_mut()
            .with_context(|| format!("candidate path is not an object at {part}: {key}"))?;
        if parts.peek().is_none() {
            object.insert(part.to_string(), replacement);
            return Ok(());
        }
        current = object
            .get_mut(part)
            .with_context(|| format!("candidate path segment not found: {part} in {key}"))?;
    }
    bail!("empty candidate key")
}

fn runtime_config_from_car_compat(config: &CarConfigCompat) -> RuntimeConfig {
    let mut runtime = RuntimeConfig {
        hz: config.control.hz,
        mapping: MappingRuntimeConfig {
            enabled: config.mapping.enabled,
            queue_capacity: config.mapping.queue_capacity,
        },
        ..Default::default()
    };
    apply_algo_overlay(&mut runtime, &config.algo);
    runtime.perception_mode = config.control.perception.mode;

    runtime.algo.perception.ransac.inlier_dist_m = config.control.perception.ransac.inlier_dist_m;
    runtime.algo.perception.ransac.min_inliers = config.control.perception.ransac.min_inliers;
    runtime.algo.perception.ransac.iterations = config.control.perception.ransac.iterations;
    runtime.algo.perception.ransac.max_walls = config.control.perception.ransac.max_walls;
    runtime.algo.perception.ransac.min_pair_sep_m = config.control.perception.ransac.min_pair_sep_m;

    runtime.algo.apex.median_window = config.control.perception.apex.median_window;
    runtime.algo.apex.min_points = config.control.perception.apex.min_points;
    runtime.algo.apex.min_forward_m = config.control.perception.apex.min_forward_m;
    runtime.algo.apex.min_range_jump_m = config.control.perception.apex.min_range_jump_m;
    runtime.algo.apex.max_opposite_dist_error_m =
        config.control.perception.apex.max_opposite_dist_error_m;
    runtime.algo.apex.prefer_nearer_opposite =
        config.control.perception.apex.prefer_nearer_opposite;
    runtime.algo.apex.wall_clearance_m = config.control.perception.apex.wall_clearance_m;
    runtime.algo.apex.apex_switch_threshold_rad =
        config.control.perception.apex.apex_switch_threshold_rad;
    runtime.algo.apex.apex_switch_hysteresis_factor =
        config.control.perception.apex.apex_switch_hysteresis_factor;
    runtime.algo.apex.max_lookahead_m = config.control.perception.apex.max_lookahead_m;
    runtime.algo.apex.min_lookahead_m = config.control.perception.apex.min_lookahead_m;
    runtime.algo.apex.lookahead_sensitivity = config.control.perception.apex.lookahead_sensitivity;
    runtime.algo.apex.side_lookahead_fov_deg =
        config.control.perception.apex.side_lookahead_fov_deg;
    runtime.algo.apex.side_lookahead_center_deg =
        config.control.perception.apex.side_lookahead_center_deg;
    runtime.algo.apex.apex_lookahead_weight = config.control.perception.apex.apex_lookahead_weight;

    runtime.algo.reactive.stanley.k_cross_track = config.control.stanley.k_cross_track;
    runtime.algo.reactive.stanley.softening_speed_ms = config.control.stanley.softening_speed_ms;
    runtime.algo.reactive.stanley.max_steering_rad = config.control.stanley.max_steering_rad;

    runtime.algo.reactive.pid.kp = config.control.pid.kp;
    runtime.algo.reactive.pid.ki = config.control.pid.ki;
    runtime.algo.reactive.pid.kd = config.control.pid.kd;
    runtime.algo.reactive.pid.windup_limit = config.control.pid.windup_limit;
    runtime.algo.reactive.pid.max_throttle = config.control.pid.max_throttle;

    runtime.algo.reactive.speed.v_max = config.control.speed.v_max;
    runtime.algo.reactive.speed.v_min = config.control.speed.v_min;
    runtime.algo.reactive.speed.k_lateral = config.control.speed.k_lateral;
    runtime.algo.reactive.speed.k_heading = config.control.speed.k_heading;
    runtime.algo.reactive.speed.obstacle_slowdown_m = config.control.speed.obstacle_slowdown_m;
    runtime.algo.reactive.speed.a_lat_max_ms2 = config.control.speed.a_lat_max_ms2;
    runtime.algo.reactive.speed.accel_limit_ms2 = config.control.speed.accel_limit_ms2;
    runtime.algo.reactive.speed.decel_limit_ms2 = config.control.speed.decel_limit_ms2;

    runtime
}

fn apply_algo_overlay(runtime: &mut RuntimeConfig, overlay: &Value) {
    if overlay.is_null() {
        return;
    }
    let mut algo = serde_json::to_value(&runtime.algo).expect("algo config serializes");
    merge_json(&mut algo, overlay);
    runtime.algo = serde_json::from_value(algo).expect("algo overlay matches runtime schema");
}

fn merge_json(base: &mut Value, overlay: &Value) {
    match (base, overlay) {
        (Value::Object(base), Value::Object(overlay)) => {
            for (key, value) in overlay {
                merge_json(base.entry(key.clone()).or_insert(Value::Null), value);
            }
        }
        (base, overlay) => *base = overlay.clone(),
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct CarConfigCompat {
    control: ControlConfigCompat,
    mapping: MappingConfigCompat,
    algo: Value,
}

impl Default for CarConfigCompat {
    fn default() -> Self {
        Self {
            control: ControlConfigCompat::default(),
            mapping: MappingConfigCompat::default(),
            algo: Value::Null,
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct MappingConfigCompat {
    enabled: bool,
    queue_capacity: usize,
}

impl Default for MappingConfigCompat {
    fn default() -> Self {
        Self {
            enabled: false,
            queue_capacity: 4,
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct ControlConfigCompat {
    hz: u64,
    speed: SpeedConfigCompat,
    pid: PidConfigCompat,
    stanley: StanleyConfigCompat,
    perception: PerceptionConfigCompat,
}

impl Default for ControlConfigCompat {
    fn default() -> Self {
        Self {
            hz: 100,
            speed: SpeedConfigCompat::default(),
            pid: PidConfigCompat {
                kp: 1.8,
                ki: 0.15,
                kd: 0.4,
                windup_limit: 0.62,
                max_throttle: 1.0,
            },
            stanley: StanleyConfigCompat::default(),
            perception: PerceptionConfigCompat::default(),
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct PerceptionConfigCompat {
    mode: PerceptionMode,
    ransac: RansacConfigCompat,
    apex: ApexConfigCompat,
}

impl Default for PerceptionConfigCompat {
    fn default() -> Self {
        Self {
            mode: PerceptionMode::Corridor,
            ransac: RansacConfigCompat::default(),
            apex: ApexConfigCompat::default(),
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct RansacConfigCompat {
    inlier_dist_m: f32,
    min_inliers: usize,
    iterations: usize,
    max_walls: usize,
    min_pair_sep_m: f32,
}

impl Default for RansacConfigCompat {
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

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct ApexConfigCompat {
    median_window: usize,
    min_points: usize,
    min_forward_m: f32,
    min_range_jump_m: f32,
    max_opposite_dist_error_m: f32,
    prefer_nearer_opposite: bool,
    wall_clearance_m: f32,
    apex_switch_threshold_rad: f32,
    apex_switch_hysteresis_factor: f32,
    max_lookahead_m: f32,
    min_lookahead_m: f32,
    lookahead_sensitivity: f32,
    side_lookahead_fov_deg: f32,
    side_lookahead_center_deg: f32,
    apex_lookahead_weight: f32,
}

impl Default for ApexConfigCompat {
    fn default() -> Self {
        Self {
            median_window: 5,
            min_points: 8,
            min_forward_m: 0.05,
            min_range_jump_m: 0.08,
            max_opposite_dist_error_m: 0.75,
            prefer_nearer_opposite: true,
            wall_clearance_m: 0.15,
            apex_switch_threshold_rad: 0.35,
            apex_switch_hysteresis_factor: 1.8,
            max_lookahead_m: 8.0,
            min_lookahead_m: 0.5,
            lookahead_sensitivity: 5.0,
            side_lookahead_fov_deg: 80.0,
            side_lookahead_center_deg: 90.0,
            apex_lookahead_weight: 0.75,
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
struct PidConfigCompat {
    kp: f32,
    ki: f32,
    kd: f32,
    windup_limit: f32,
    max_throttle: f32,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct SpeedConfigCompat {
    v_max: f32,
    v_min: f32,
    k_lateral: f32,
    k_heading: f32,
    obstacle_slowdown_m: f32,
    a_lat_max_ms2: f32,
    accel_limit_ms2: f32,
    decel_limit_ms2: f32,
}

impl Default for SpeedConfigCompat {
    fn default() -> Self {
        Self {
            v_max: 1.8,
            v_min: 0.35,
            k_lateral: 2.0,
            k_heading: 2.0,
            obstacle_slowdown_m: 3.0,
            a_lat_max_ms2: 3.0,
            accel_limit_ms2: 1.5,
            decel_limit_ms2: 5.0,
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default)]
struct StanleyConfigCompat {
    k_cross_track: f32,
    softening_speed_ms: f32,
    max_steering_rad: f32,
}

impl Default for StanleyConfigCompat {
    fn default() -> Self {
        Self {
            k_cross_track: 1.0,
            softening_speed_ms: 1.0,
            max_steering_rad: 0.38,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(params: &[(&str, f64)]) -> Candidate {
        Candidate {
            params: params
                .iter()
                .map(|(key, value)| ((*key).to_string(), *value))
                .collect(),
        }
    }

    #[test]
    fn candidate_serializes_round_trip() {
        let candidate = candidate(&[("algo.apex.min_range_jump_m", 0.2)]);

        let json = serde_json::to_string(&candidate).unwrap();
        let decoded: Candidate = serde_json::from_str(&json).unwrap();

        assert_eq!(decoded, candidate);
    }

    #[test]
    fn candidate_applies_flat_overrides_without_resetting_skipped_fields() {
        let mut base = RuntimeConfig::default();
        base.algo.apex.prefer_nearer_opposite = false;
        let candidate = candidate(&[
            ("algo.apex.min_range_jump_m", 0.2),
            ("algo.apex.median_window", 4.7),
        ]);

        let tuned = candidate.apply_to_runtime_config(&base).unwrap();

        assert_eq!(tuned.algo.apex.min_range_jump_m, 0.2);
        assert_eq!(tuned.algo.apex.median_window, 5);
        assert!(!tuned.algo.apex.prefer_nearer_opposite);
    }

    #[test]
    fn validation_rejects_inverted_apex_lookahead_bounds() {
        let mut config = RuntimeConfig::default();
        config.algo.apex.min_lookahead_m = 4.5;
        config.algo.apex.max_lookahead_m = 2.8;

        let errors = validate_runtime_config(&config);

        assert_eq!(errors.len(), 1);
        assert!(errors[0].contains("min_lookahead_m"));
    }

    #[test]
    fn car_config_conversion_preserves_all_apex_fields() {
        let mut car = CarConfigCompat::default();
        car.control.perception.mode = PerceptionMode::Apex;
        car.control.perception.apex = ApexConfigCompat {
            median_window: 7,
            min_points: 9,
            min_forward_m: 0.11,
            min_range_jump_m: 0.21,
            max_opposite_dist_error_m: 0.31,
            prefer_nearer_opposite: false,
            wall_clearance_m: 0.12,
            apex_switch_threshold_rad: 0.22,
            apex_switch_hysteresis_factor: 2.4,
            max_lookahead_m: 7.0,
            min_lookahead_m: 0.7,
            lookahead_sensitivity: 3.3,
            side_lookahead_fov_deg: 55.0,
            side_lookahead_center_deg: 88.0,
            apex_lookahead_weight: 0.6,
        };

        let runtime = runtime_config_from_car_compat(&car);
        let json = serde_json::to_string(&runtime).unwrap();
        let reloaded: RuntimeConfig = serde_json::from_str(&json).unwrap();

        assert_eq!(reloaded.perception_mode, PerceptionMode::Apex);
        assert_eq!(reloaded.algo.apex.median_window, 7);
        assert_eq!(reloaded.algo.apex.min_points, 9);
        assert_eq!(reloaded.algo.apex.min_forward_m, 0.11);
        assert_eq!(reloaded.algo.apex.min_range_jump_m, 0.21);
        assert_eq!(reloaded.algo.apex.max_opposite_dist_error_m, 0.31);
        assert!(!reloaded.algo.apex.prefer_nearer_opposite);
        assert_eq!(reloaded.algo.apex.wall_clearance_m, 0.12);
        assert_eq!(reloaded.algo.apex.apex_switch_threshold_rad, 0.22);
        assert_eq!(reloaded.algo.apex.apex_switch_hysteresis_factor, 2.4);
        assert_eq!(reloaded.algo.apex.max_lookahead_m, 7.0);
        assert_eq!(reloaded.algo.apex.min_lookahead_m, 0.7);
        assert_eq!(reloaded.algo.apex.lookahead_sensitivity, 3.3);
        assert_eq!(reloaded.algo.apex.side_lookahead_fov_deg, 55.0);
        assert_eq!(reloaded.algo.apex.side_lookahead_center_deg, 88.0);
        assert_eq!(reloaded.algo.apex.apex_lookahead_weight, 0.6);
    }

    #[test]
    fn car_config_conversion_preserves_mapping_runtime_fields() {
        let mut car = CarConfigCompat::default();
        car.mapping.enabled = true;
        car.mapping.queue_capacity = 7;

        let runtime = runtime_config_from_car_compat(&car);

        assert!(runtime.mapping.enabled);
        assert_eq!(runtime.mapping.queue_capacity, 7);
    }

    #[test]
    fn car_config_conversion_merges_algo_overlay() {
        let car = CarConfigCompat {
            algo: serde_json::json!({
                "mapper": {
                    "submap_translation_m": 0.75,
                    "submap_yaw_rad": 0.25
                },
                "particle": {
                    "resample_ess_fraction": 0.8,
                    "recovery_fraction": 0.4,
                    "min_confidence": 0.2
                }
            }),
            ..CarConfigCompat::default()
        };

        let runtime = runtime_config_from_car_compat(&car);

        assert_eq!(runtime.algo.mapper.submap_translation_m, 0.75);
        assert_eq!(runtime.algo.mapper.submap_yaw_rad, 0.25);
        assert_eq!(runtime.algo.particle.resample_ess_fraction, 0.8);
        assert_eq!(runtime.algo.particle.recovery_fraction, 0.4);
        assert_eq!(runtime.algo.particle.min_confidence, 0.2);
    }
}
