/// control/watchdog.rs — software watchdog for the control loop
///
/// The control loop calls `kick()` every tick.
/// If the loop overruns (tick took longer than CONTROL_PERIOD), it calls `miss()`.
/// After WATCHDOG_MAX_MISSED consecutive misses, the caller engages safe_state.
///
/// Additionally, `kick()` calls sd_notify(WATCHDOG=1) so systemd's WatchdogSec=5s
/// timer is reset.  If the process hangs entirely, systemd will restart it.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use libsystemd::daemon::{self, NotifyState};

pub const WATCHDOG_MAX_MISSED: u32 = 3;

pub struct Watchdog {
    missed: Arc<AtomicU32>,
}

impl Watchdog {
    pub fn new() -> Self {
        Self { missed: Arc::new(AtomicU32::new(0)) }
    }

    /// Appeler a chaque tick reussi
    pub fn kick(&self) {
        self.missed.store(0, Ordering::Relaxed);
        let _ = daemon::notify(false, &[NotifyState::Watchdog]);
    }

    /// Appeler lors d'un un miss
    pub fn miss(&self) -> u32 {
        self.missed.fetch_add(1, Ordering::Relaxed) + 1
    }
}
