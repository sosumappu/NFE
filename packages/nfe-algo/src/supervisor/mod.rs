//! Mode-switching supervisor.
//!
//! Decoupled from control laws and estimators: it consumes a `HealthReport`
//! each tick and emits a `DriveMode`. It does not know LQR from MPC or EKF from
//! particle filter. Safety (ESTOP/blind/watchdog) sits below this and can
//! override any mode to safe-state — that is a separate layer.
//!
//! Policy (race of 5 laps, unattended):
//!   - Lap 1 always drives REACTIVE while the mapping task builds the map.
//!   - Lap completion is signalled by the physical start/finish line (orange
//!     ground line + laser timing gates) via `start_line_crossed`, NOT by
//!     geometric loop closure. Geometric scan-match loop closure feeds the
//!     map-quality health gate only.
//!   - After the map is ready (lap >= 1 complete, loop-closure quality good,
//!     raceline computed) the supervisor engages RACELINE — but only after a
//!     dwell of consecutive healthy ticks (hysteresis), and only if mapping is
//!     enabled at all.
//!   - At any time, if localization confidence drops or the estimator diverges
//!     for a sustained window, it falls back to REACTIVE immediately. Re-engage
//!     requires passing the dwell again, so it cannot flap on a single bad scan.

use nfe_core::params::Tunable;

/// Which control strategy the pipeline should drive this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveMode {
    /// Reactive wall-following (lap 1 + safety fallback).
    Reactive,
    /// Map-based localization + precomputed racing line (laps 2-5).
    RaceLine,
}

/// Quality of the geometric scan-match loop closure (map consistency check).
#[derive(Clone, Copy, Debug)]
pub struct LoopClosureStatus {
    /// True once the accumulated map closes on itself geometrically.
    pub detected: bool,
    /// Alignment residual at closure [m]; lower is better.
    pub residual_m: f32,
    /// Fraction of scan overlapping the existing map in [0,1].
    pub overlap: f32,
}

impl Default for LoopClosureStatus {
    fn default() -> Self {
        Self {
            detected: false,
            residual_m: f32::INFINITY,
            overlap: 0.0,
        }
    }
}

/// Everything the supervisor needs to decide a mode. Produced by the pipeline
/// from the estimator, mapper, and start-line sensor.
#[derive(Clone, Copy, Debug)]
pub struct HealthReport {
    /// Localization confidence in [0,1] (EKF/scan-match). Drives fallback.
    pub localization_confidence: f32,
    /// Geometric loop-closure status (map quality check, not lap trigger).
    pub loop_closure: LoopClosureStatus,
    /// Estimator divergence flag (e.g. covariance blew up / NIS gate tripped).
    pub estimator_diverged: bool,
    /// A precomputed racing line is available to track.
    pub raceline_ready: bool,
    /// Mapping subsystem is enabled in config (independent of mode).
    pub mapping_enabled: bool,
    /// Rising edge: the car crossed the physical start/finish line this tick.
    pub start_line_crossed: bool,
}

impl Default for HealthReport {
    fn default() -> Self {
        Self {
            localization_confidence: 0.0,
            loop_closure: LoopClosureStatus::default(),
            estimator_diverged: false,
            raceline_ready: false,
            mapping_enabled: true,
            start_line_crossed: false,
        }
    }
}

/// Tunable thresholds and dwell windows.
#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct SupervisorParams {
    /// Minimum localization confidence to stay engaged in RaceLine.
    #[param(0.1..0.95, default = 0.55)]
    pub min_localization_confidence: f32,
    /// Maximum acceptable loop-closure residual [m] for map to count as good.
    #[param(0.02..0.5, default = 0.12)]
    pub max_loop_closure_residual_m: f32,
    /// Minimum scan/map overlap for the map to count as good.
    #[param(0.1..0.9, default = 0.4)]
    pub min_loop_closure_overlap: f32,
    /// Consecutive healthy ticks required before engaging RaceLine.
    #[param(int, 1..200, default = 25)]
    pub engage_dwell_ticks: u32,
    /// Consecutive unhealthy ticks required before falling back to Reactive.
    #[param(int, 1..100, default = 5)]
    pub fallback_dwell_ticks: u32,
    /// Lap on which RaceLine may first engage (1 = after first crossing).
    #[param(int, 1..5, default = 1)]
    pub min_lap_for_raceline: u32,
}

impl Default for SupervisorParams {
    fn default() -> Self {
        Self {
            min_localization_confidence: 0.55,
            max_loop_closure_residual_m: 0.12,
            min_loop_closure_overlap: 0.4,
            engage_dwell_ticks: 25,
            fallback_dwell_ticks: 5,
            min_lap_for_raceline: 1,
        }
    }
}

/// The supervisor state machine.
pub struct RaceSupervisor {
    params: SupervisorParams,
    mode: DriveMode,
    /// Laps completed = number of start-line crossings observed.
    lap: u32,
    /// Consecutive healthy ticks (for engage hysteresis).
    healthy_streak: u32,
    /// Consecutive unhealthy ticks (for fallback hysteresis).
    unhealthy_streak: u32,
    /// Latches once the map has ever been judged good, so transient overlap
    /// dips after engagement don't retract map-readiness (only confidence /
    /// divergence drive fallback once engaged).
    map_quality_ok: bool,
}

impl RaceSupervisor {
    pub fn new(params: SupervisorParams) -> Self {
        Self {
            params,
            mode: DriveMode::Reactive,
            lap: 0,
            healthy_streak: 0,
            unhealthy_streak: 0,
            map_quality_ok: false,
        }
    }

    pub fn mode(&self) -> DriveMode {
        self.mode
    }

    pub fn lap(&self) -> u32 {
        self.lap
    }

    /// True when the geometric loop closure currently meets quality thresholds.
    fn loop_closure_good(&self, h: &HealthReport) -> bool {
        let lc = h.loop_closure;
        lc.detected
            && lc.residual_m <= self.params.max_loop_closure_residual_m
            && lc.overlap >= self.params.min_loop_closure_overlap
    }

    /// Advance one tick. Returns the mode the pipeline should drive now.
    pub fn step(&mut self, h: &HealthReport) -> DriveMode {
        // Lap accounting is driven purely by the physical start-line signal.
        if h.start_line_crossed {
            self.lap = self.lap.saturating_add(1);
        }

        // Map-quality latch: once loop closure is good, remember it. Mapping
        // disabled => map can never be considered good.
        if h.mapping_enabled && self.loop_closure_good(h) {
            self.map_quality_ok = true;
        }
        if !h.mapping_enabled {
            self.map_quality_ok = false;
        }

        // Conditions that make RaceLine *eligible* this tick.
        let eligible = h.mapping_enabled
            && h.raceline_ready
            && self.map_quality_ok
            && self.lap >= self.params.min_lap_for_raceline;

        // Healthy = eligible AND localization is trustworthy AND not diverged.
        let healthy = eligible
            && !h.estimator_diverged
            && h.localization_confidence >= self.params.min_localization_confidence;

        if healthy {
            self.healthy_streak = self.healthy_streak.saturating_add(1);
            self.unhealthy_streak = 0;
        } else {
            self.unhealthy_streak = self.unhealthy_streak.saturating_add(1);
            self.healthy_streak = 0;
        }

        self.mode = match self.mode {
            DriveMode::Reactive => {
                if healthy && self.healthy_streak >= self.params.engage_dwell_ticks {
                    DriveMode::RaceLine
                } else {
                    DriveMode::Reactive
                }
            }
            DriveMode::RaceLine => {
                // Fall back when unhealthy for the fallback dwell. Note that
                // "unhealthy" includes a confidence drop or divergence; it does
                // NOT include a transient overlap dip because map_quality_ok is
                // latched once the map is built.
                if !healthy && self.unhealthy_streak >= self.params.fallback_dwell_ticks {
                    DriveMode::Reactive
                } else {
                    DriveMode::RaceLine
                }
            }
        };

        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_report() -> HealthReport {
        HealthReport {
            localization_confidence: 0.9,
            loop_closure: LoopClosureStatus {
                detected: true,
                residual_m: 0.05,
                overlap: 0.7,
            },
            estimator_diverged: false,
            raceline_ready: true,
            mapping_enabled: true,
            start_line_crossed: false,
        }
    }

    #[test]
    fn starts_in_reactive() {
        let s = RaceSupervisor::new(SupervisorParams::default());
        assert_eq!(s.mode(), DriveMode::Reactive);
    }

    #[test]
    fn start_line_increments_lap() {
        let mut s = RaceSupervisor::new(SupervisorParams::default());
        let mut h = HealthReport {
            start_line_crossed: true,
            ..HealthReport::default()
        };
        s.step(&h);
        assert_eq!(s.lap(), 1);
        h.start_line_crossed = false;
        s.step(&h);
        assert_eq!(s.lap(), 1);
    }

    #[test]
    fn engages_raceline_only_after_dwell() {
        let p = SupervisorParams {
            engage_dwell_ticks: 10,
            min_lap_for_raceline: 1,
            ..SupervisorParams::default()
        };
        let mut s = RaceSupervisor::new(p);

        // Cross the start line once to satisfy lap requirement. This tick is
        // itself healthy, so it counts as the first of the engage dwell.
        let mut h = healthy_report();
        h.start_line_crossed = true;
        assert_eq!(s.step(&h), DriveMode::Reactive); // healthy_streak = 1
        h.start_line_crossed = false;

        // 8 more healthy ticks keep us Reactive (streak 2..9).
        for _ in 0..8 {
            assert_eq!(s.step(&h), DriveMode::Reactive);
        }
        // 10th healthy tick reaches the dwell and engages.
        assert_eq!(s.step(&h), DriveMode::RaceLine);
    }

    #[test]
    fn does_not_engage_before_lap_requirement() {
        let mut s = RaceSupervisor::new(SupervisorParams {
            engage_dwell_ticks: 1,
            min_lap_for_raceline: 1,
            ..SupervisorParams::default()
        });
        let h = healthy_report(); // healthy but no start-line crossing yet
        for _ in 0..50 {
            assert_eq!(s.step(&h), DriveMode::Reactive);
        }
    }

    #[test]
    fn falls_back_on_sustained_low_confidence() {
        let mut s = RaceSupervisor::new(SupervisorParams {
            engage_dwell_ticks: 1,
            fallback_dwell_ticks: 3,
            min_lap_for_raceline: 1,
            ..SupervisorParams::default()
        });
        let mut h = healthy_report();
        h.start_line_crossed = true;
        assert_eq!(s.step(&h), DriveMode::RaceLine);
        h.start_line_crossed = false;
        assert_eq!(s.step(&h), DriveMode::RaceLine);

        // Confidence collapses.
        h.localization_confidence = 0.1;
        assert_eq!(s.step(&h), DriveMode::RaceLine); // streak 1
        assert_eq!(s.step(&h), DriveMode::RaceLine); // streak 2
        assert_eq!(s.step(&h), DriveMode::Reactive); // streak 3 -> fallback
    }

    #[test]
    fn single_bad_tick_does_not_flap() {
        let mut s = RaceSupervisor::new(SupervisorParams {
            engage_dwell_ticks: 1,
            fallback_dwell_ticks: 5,
            min_lap_for_raceline: 1,
            ..SupervisorParams::default()
        });
        let mut h = healthy_report();
        h.start_line_crossed = true;
        assert_eq!(s.step(&h), DriveMode::RaceLine);
        h.start_line_crossed = false;

        // One bad tick then recovery: must stay RaceLine.
        h.localization_confidence = 0.0;
        assert_eq!(s.step(&h), DriveMode::RaceLine);
        h.localization_confidence = 0.9;
        for _ in 0..10 {
            assert_eq!(s.step(&h), DriveMode::RaceLine);
        }
    }

    #[test]
    fn divergence_forces_fallback() {
        let mut s = RaceSupervisor::new(SupervisorParams {
            engage_dwell_ticks: 1,
            fallback_dwell_ticks: 1,
            min_lap_for_raceline: 1,
            ..SupervisorParams::default()
        });
        let mut h = healthy_report();
        h.start_line_crossed = true;
        assert_eq!(s.step(&h), DriveMode::RaceLine);
        h.start_line_crossed = false;
        h.estimator_diverged = true;
        assert_eq!(s.step(&h), DriveMode::Reactive);
    }

    #[test]
    fn mapping_disabled_never_engages() {
        let mut s = RaceSupervisor::new(SupervisorParams {
            engage_dwell_ticks: 1,
            min_lap_for_raceline: 1,
            ..SupervisorParams::default()
        });
        let mut h = healthy_report();
        h.mapping_enabled = false;
        h.start_line_crossed = true;
        for _ in 0..50 {
            assert_eq!(s.step(&h), DriveMode::Reactive);
        }
    }
}
