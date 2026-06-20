use crate::hal::SensorSource;
/// replay/live_source.rs — Live SensorSource backed by SharedState
///
/// Implements `SensorSource` on top of the existing `SharedState` + thread
/// architecture.  Optionally forwards every sensor update to a `Recorder`
/// so that sessions can be replayed offline without any changes to the
/// sensor threads themselves.
use crate::state::{SensorSnapshot, SharedState};
use anyhow::Result;
use std::sync::Arc;

/// Wraps `SharedState` and implements `SensorSource` for the control loop.
pub struct LiveSensorSource {
    state: Arc<SharedState>,
}

impl LiveSensorSource {
    pub fn new(state: Arc<SharedState>) -> Self {
        Self { state }
    }
}

impl SensorSource for LiveSensorSource {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        Ok(self.state.snapshot())
    }

    fn is_exhausted(&self) -> bool {
        false
    }
}
