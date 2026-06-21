//! Tunable parameter registry.
//!
//! Every module owns a `#[derive(Tunable)]` param struct. The derive flattens
//! each struct into dotted, namespaced descriptors (e.g. `control.lqr.k_lat`)
//! with bounds and scale, so config loading and optimizers share one source of
//! truth. There is no hand-written `to_vec`/`from_slice`/`clamp` per tuner.

use std::collections::HashMap;

pub use nfe_tunable_derive::Tunable;

/// Search-space descriptor for a single scalar parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamSpec {
    /// Continuous parameter. `log` selects log-scale sampling for optimizers.
    Continuous {
        lo: f64,
        hi: f64,
        default: f64,
        log: bool,
    },
    /// Integer parameter (e.g. filter window size, clean-tick count).
    Integer { lo: i64, hi: i64, default: i64 },
}

impl ParamSpec {
    /// Default value as f64 regardless of variant.
    pub fn default_value(&self) -> f64 {
        match *self {
            ParamSpec::Continuous { default, .. } => default,
            ParamSpec::Integer { default, .. } => default as f64,
        }
    }

    /// Clamp a candidate to the spec's bounds. Integer specs also round.
    pub fn clamp(&self, v: f64) -> f64 {
        match *self {
            ParamSpec::Continuous { lo, hi, .. } => v.clamp(lo, hi),
            ParamSpec::Integer { lo, hi, .. } => (v.round() as i64).clamp(lo, hi) as f64,
        }
    }

    /// Lower/upper bounds as f64 (integer hi is inclusive at the i64 level).
    pub fn bounds(&self) -> (f64, f64) {
        match *self {
            ParamSpec::Continuous { lo, hi, .. } => (lo, hi),
            ParamSpec::Integer { lo, hi, .. } => (lo as f64, hi as f64),
        }
    }
}

/// Reflect a param struct into a flat search space and back.
///
/// Implemented via `#[derive(Tunable)]`. `prefix` is the namespace path of the
/// containing struct; the derive appends `.<field>` for each member and
/// recurses into `#[tunable(nested)]` fields.
pub trait Tunable {
    /// Append `(key, spec)` descriptors for every tunable scalar under `prefix`.
    fn descriptors(prefix: &str, out: &mut Vec<(String, ParamSpec)>);

    /// Append `(key, value)` for the current values under `prefix`.
    fn to_flat(&self, prefix: &str, out: &mut Vec<(String, f64)>);

    /// Reconstruct from a flat value map. Missing keys use declared defaults.
    fn from_flat(prefix: &str, values: &HashMap<String, f64>) -> Self;
}

/// Convenience helpers usable on any `Tunable` without restating the prefix.
pub trait TunableExt: Tunable + Sized {
    /// Full descriptor list rooted at `root` (e.g. "" or "control").
    fn search_space(root: &str) -> Vec<(String, ParamSpec)> {
        let mut out = Vec::new();
        Self::descriptors(root, &mut out);
        out
    }

    /// Current values flattened, rooted at `root`.
    fn flatten(&self, root: &str) -> Vec<(String, f64)> {
        let mut out = Vec::new();
        self.to_flat(root, &mut out);
        out
    }

    /// Build from a flat optimizer vector aligned to `search_space(root)`,
    /// clamping each value to its spec before reconstruction.
    fn from_vec(root: &str, vector: &[f64]) -> Self {
        let specs = Self::search_space(root);
        let mut map = HashMap::with_capacity(specs.len());
        for (i, (key, spec)) in specs.iter().enumerate() {
            let raw = vector
                .get(i)
                .copied()
                .unwrap_or_else(|| spec.default_value());
            map.insert(key.clone(), spec.clamp(raw));
        }
        Self::from_flat(root, &map)
    }
}

impl<T: Tunable + Sized> TunableExt for T {}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Tunable, Default)]
    struct Inner {
        #[param(0.0..4.0, default = 0.8)]
        k_lat: f32,
        #[param(int, 1..21, default = 5)]
        window: usize,
    }

    #[derive(Tunable, Default)]
    struct Outer {
        #[tunable(nested)]
        inner: Inner,
        #[param(1e-3..1e1, default = 0.05, log)]
        ki: f32,
        #[tunable(skip)]
        _scratch: f32,
    }

    #[test]
    fn descriptors_are_namespaced_and_typed() {
        let specs = Outer::search_space("control");
        let keys: Vec<_> = specs.iter().map(|(k, _)| k.as_str()).collect();
        assert!(keys.contains(&"control.inner.k_lat"));
        assert!(keys.contains(&"control.inner.window"));
        assert!(keys.contains(&"control.ki"));

        let window = specs
            .iter()
            .find(|(k, _)| k == "control.inner.window")
            .map(|(_, s)| *s)
            .unwrap();
        assert!(matches!(
            window,
            ParamSpec::Integer {
                lo: 1,
                hi: 21,
                default: 5
            }
        ));

        let ki = specs
            .iter()
            .find(|(k, _)| k == "control.ki")
            .map(|(_, s)| *s)
            .unwrap();
        assert!(matches!(ki, ParamSpec::Continuous { log: true, .. }));
    }

    #[test]
    fn roundtrip_through_flat_vector() {
        let specs = Outer::search_space("control");
        // Build a vector that violates bounds; from_vec must clamp/round it.
        let raw: Vec<f64> = specs
            .iter()
            .map(|(k, _)| match k.as_str() {
                "control.inner.k_lat" => 99.0, // over hi=4.0 -> clamp
                "control.inner.window" => 7.6, // -> round to 8
                "control.ki" => 0.02,
                _ => 0.0,
            })
            .collect();

        let rebuilt = Outer::from_vec("control", &raw);
        assert_eq!(rebuilt.inner.k_lat, 4.0);
        assert_eq!(rebuilt.inner.window, 8);
        assert!((rebuilt.ki - 0.02).abs() < 1e-6);
    }

    #[test]
    fn missing_keys_fall_back_to_defaults() {
        let map = HashMap::new(); // empty
        let rebuilt = Outer::from_flat("control", &map);
        assert_eq!(rebuilt.inner.k_lat, 0.8);
        assert_eq!(rebuilt.inner.window, 5);
        assert!((rebuilt.ki - 0.05).abs() < 1e-6);
    }

    #[test]
    fn empty_prefix_does_not_emit_leading_dot() {
        let specs = Outer::search_space("");
        let keys: Vec<_> = specs.iter().map(|(k, _)| k.as_str()).collect();
        assert!(keys.contains(&"inner.k_lat"));
        assert!(keys.contains(&"inner.window"));
        assert!(keys.contains(&"ki"));
        assert!(!keys.iter().any(|k| k.starts_with('.')));
    }

    #[test]
    fn from_flat_rounds_integer_fields() {
        let mut map = HashMap::new();
        map.insert("control.inner.window".to_string(), 7.6);
        let rebuilt = Outer::from_flat("control", &map);
        assert_eq!(rebuilt.inner.window, 8);
    }
}
