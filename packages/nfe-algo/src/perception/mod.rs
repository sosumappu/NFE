//! Perception: turn a LIDAR cloud into walls and a track error signal.
//!
//! `ransac` is the shared wall-extraction path used by both the reactive
//! controller and the mapping task. `track_error` (handoff item) consumes the
//! fitted walls to produce the crosstrack/heading error the reactive
//! controller tracks.

pub mod corridor;
pub mod ransac;
