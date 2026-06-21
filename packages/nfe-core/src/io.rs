//! Runtime I/O trait boundaries shared by live, sim, replay, and tests.
//!
//! These traits are intentionally tiny and contain no bus, file, network, or
//! hardware details. Concrete crates decide how to implement them.

use crate::sensors::SensorSnapshot;

pub trait SensorSource {
    fn next_snapshot(&mut self) -> anyhow::Result<Option<SensorSnapshot>>;
}

pub trait ActuatorSink {
    fn apply(&mut self, output: &crate::control::ControlOutput) -> anyhow::Result<()>;
    fn safe_state(&mut self) -> anyhow::Result<()>;
}
