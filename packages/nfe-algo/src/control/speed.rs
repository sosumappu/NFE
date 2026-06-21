use nfe_core::control::CorridorEstimate;
use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct SpeedParams {
    #[param(0.1..10.0, default = 1.0)]
    pub v_max: f32,
    #[param(0.0..10.0, default = 1.0)]
    pub k_heading: f32,
    #[param(0.0..10.0, default = 1.0)]
    pub k_lateral: f32,
    #[param(0.05..5.0, default = 0.4)]
    pub obstacle_slowdown_m: f32,
}

impl Default for SpeedParams {
    fn default() -> Self {
        Self {
            v_max: 1.0,
            k_heading: 1.0,
            k_lateral: 1.0,
            obstacle_slowdown_m: 0.4,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SpeedPlanner {
    params: SpeedParams,
}

impl SpeedPlanner {
    pub fn new(params: SpeedParams) -> Self {
        Self { params }
    }

    pub fn compute(&self, corridor: Option<&CorridorEstimate>) -> f32 {
        let Some(c) = corridor else {
            return 0.0;
        };
        let heading_factor =
            (1.0 - self.params.k_heading * c.heading_error_rad.powi(2)).clamp(0.0, 1.0);
        let lateral_factor =
            (1.0 - self.params.k_lateral * c.lateral_error_m.abs()).clamp(0.0, 1.0);
        let obstacle_factor = if c.nearest_obstacle_m.is_finite() {
            (c.nearest_obstacle_m / self.params.obstacle_slowdown_m).clamp(0.0, 1.0)
        } else {
            1.0
        };
        self.params.v_max
            * heading_factor
            * lateral_factor
            * obstacle_factor
            * c.confidence.clamp(0.0, 1.0)
    }
}
