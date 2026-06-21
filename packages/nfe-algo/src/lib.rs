//! `nfe-algo` — pure algorithms with no I/O, no tokio, no hardware.
//!
//! These modules are deterministic and unit-testable in isolation; the runtime
//! crate wires them to sensors, actuators, and the async control loop. Linking
//! only `nfe-algo` (+`nfe-core`) keeps the tuner and tests free of tokio.
//!
//! Critical pieces implemented here: `perception::ransac` (shared wall fitter),
//! `estimation::ekf` (pose + IMU-bias filter), `supervisor` (mode state machine
//! with health-gate/hysteresis + start-line trigger). Remaining modules
//! (`StateEstimator`/`Controller` traits, mapper, localizer, raceline) are
//! handoff items mirroring these as structural references.

pub mod config;
pub mod control;
pub mod estimation;
pub mod localization;
pub mod mapping;
pub mod perception;
pub mod raceline;
pub mod supervisor;
