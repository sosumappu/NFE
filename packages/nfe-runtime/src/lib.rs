//! `nfe-runtime` — orchestration around pure NFE algorithms.
//!
//! This crate intentionally keeps hardware/Tokio specifics thin. The central
//! piece is `Pipeline::step`, a deterministic synchronous tick used by live,
//! replay, sim, and the tuner.

pub mod config;
pub mod foxglove_scenes;
pub mod input_replay;
pub mod mapping_worker;
pub mod pipeline;
pub mod raceline_preview;
pub mod raceline_worker;
pub mod run_loop;
pub mod session;
pub mod sinks;
pub mod start_gate;
pub mod start_line;
pub mod telemetry_bus;
pub mod tuning;
