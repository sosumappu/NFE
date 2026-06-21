//! State estimation. `ekf` is the full pose+bias filter used by the map-based
//! path. The `StateEstimator` trait (handoff item) abstracts over the EKF and
//! the cheap dead-reckon estimator so the reactive path pays no SLAM tax.

pub mod dead_reckon;
pub mod ekf;
