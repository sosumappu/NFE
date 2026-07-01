//! Raceline generation and tracking.

pub mod controller;
pub mod solver;

pub(crate) mod contour;
pub(crate) mod edt;
pub(crate) mod geometry;
pub(crate) mod grid;
pub mod lateral;
pub mod longitudinal;
pub(crate) mod min_curvature;
pub(crate) mod qp;
pub(crate) mod reference;
pub(crate) mod savgol;
pub(crate) mod spline;
pub mod steering;
pub mod velocity;
pub(crate) mod watershed;
