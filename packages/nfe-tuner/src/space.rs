use nfe_core::params::ParamSpec;

#[derive(Clone, Copy, Debug, serde::Deserialize, serde::Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Scale {
    Linear,
    Log,
}

#[derive(Clone, Copy, Debug, serde::Deserialize, serde::Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ParamKind {
    Float,
    Int,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, PartialEq)]
pub struct SearchSpaceEntry {
    pub name: String,
    pub low: f64,
    pub high: f64,
    pub scale: Scale,
    pub kind: ParamKind,
}

impl SearchSpaceEntry {
    pub fn from_param_spec(name: String, spec: ParamSpec) -> Self {
        let mut entry = match spec {
            ParamSpec::Continuous { lo, hi, log, .. } => Self {
                name,
                low: lo,
                high: hi,
                scale: if log { Scale::Log } else { Scale::Linear },
                kind: ParamKind::Float,
            },
            ParamSpec::Integer { lo, hi, .. } => Self {
                name,
                low: lo as f64,
                high: hi as f64,
                scale: Scale::Linear,
                kind: ParamKind::Int,
            },
        };
        apply_search_overrides(&mut entry);
        entry
    }
}

fn apply_search_overrides(entry: &mut SearchSpaceEntry) {
    match entry.name.as_str() {
        "algo.apex.min_range_jump_m" | "algo.apex.max_opposite_dist_error_m" => {
            entry.low = 0.01;
            entry.high = 2.0;
            entry.scale = Scale::Log;
        }
        "algo.apex.side_lookahead_fov_deg" => {
            entry.low = 10.0;
            entry.high = 120.0;
        }
        "algo.apex.side_lookahead_center_deg" => {
            entry.low = 45.0;
            entry.high = 135.0;
        }
        "algo.apex.apex_lookahead_weight" => {
            entry.low = 0.0;
            entry.high = 1.0;
        }
        _ => {}
    }
}

pub fn search_space_entries(prefixes: &[String]) -> Vec<SearchSpaceEntry> {
    nfe_runtime::tuning::search_space()
        .into_iter()
        .filter(|(name, _)| prefixes.is_empty() || prefixes.iter().any(|p| name.starts_with(p)))
        .map(|(name, spec)| SearchSpaceEntry::from_param_spec(name, spec))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serializes_round_trip() {
        let entry = SearchSpaceEntry {
            name: "algo.apex.min_range_jump_m".to_string(),
            low: 0.01,
            high: 2.0,
            scale: Scale::Log,
            kind: ParamKind::Float,
        };

        let json = serde_json::to_string(&entry).unwrap();
        assert!(json.contains("\"scale\":\"log\""));
        let decoded: SearchSpaceEntry = serde_json::from_str(&json).unwrap();

        assert_eq!(decoded, entry);
    }

    #[test]
    fn filters_by_prefix() {
        let entries = search_space_entries(&["algo.apex".to_string()]);

        assert!(!entries.is_empty());
        assert!(entries
            .iter()
            .all(|entry| entry.name.starts_with("algo.apex")));
    }

    #[test]
    fn applies_apex_search_overrides() {
        let entries = search_space_entries(&["algo.apex".to_string()]);
        let by_name = |name: &str| entries.iter().find(|entry| entry.name == name).unwrap();

        let min_jump = by_name("algo.apex.min_range_jump_m");
        assert_eq!(min_jump.scale, Scale::Log);
        assert_eq!(min_jump.low, 0.01);
        assert_eq!(min_jump.high, 2.0);

        let opposite = by_name("algo.apex.max_opposite_dist_error_m");
        assert_eq!(opposite.scale, Scale::Log);
        assert_eq!(opposite.low, 0.01);
        assert_eq!(opposite.high, 2.0);

        assert_eq!(by_name("algo.apex.side_lookahead_fov_deg").low, 10.0);
        assert_eq!(by_name("algo.apex.side_lookahead_fov_deg").high, 120.0);
        assert_eq!(by_name("algo.apex.side_lookahead_center_deg").low, 45.0);
        assert_eq!(by_name("algo.apex.side_lookahead_center_deg").high, 135.0);
        assert_eq!(by_name("algo.apex.apex_lookahead_weight").low, 0.0);
        assert_eq!(by_name("algo.apex.apex_lookahead_weight").high, 1.0);
    }
}
