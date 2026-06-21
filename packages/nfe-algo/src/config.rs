//! Typed pure-algorithm config. Runtime config embeds this and adds I/O details.

use nfe_core::params::Tunable;

use crate::control::reactive::ReactiveControllerParams;
use crate::estimation::ekf::EkfParams;
use crate::localization::particle::ParticleParams;
use crate::localization::scan_match::ScanMatchParams;
use crate::mapping::MapperParams;
use crate::perception::corridor::CorridorParams;
use crate::raceline::controller::RaceLineControllerParams;
use crate::raceline::solver::RaceLineSolverParams;
use crate::supervisor::SupervisorParams;

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct AlgoConfig {
    #[tunable(nested)]
    pub ekf: EkfParams,
    #[tunable(nested)]
    pub perception: CorridorParams,
    #[tunable(nested)]
    pub mapper: MapperParams,
    #[tunable(nested)]
    pub scan_match: ScanMatchParams,
    #[tunable(nested)]
    pub particle: ParticleParams,
    #[tunable(nested)]
    pub supervisor: SupervisorParams,
    #[tunable(nested)]
    pub reactive: ReactiveControllerParams,
    #[tunable(nested)]
    pub raceline_controller: RaceLineControllerParams,
    #[tunable(nested)]
    pub raceline_solver: RaceLineSolverParams,
}
