/// control/watchdog.rs — software watchdog for the control loop
///
/// The control loop calls `kick()` every tick.
/// If the loop overruns (tick took longer than CONTROL_PERIOD), it calls `miss()`.
/// After WATCHDOG_MAX_MISSED consecutive misses, the caller engages safe_state.
///
/// Additionally, `kick()` calls sd_notify(WATCHDOG=1) so systemd's WatchdogSec=5s
/// timer is reset.  If the process hangs entirely, systemd will restart it.
#[cfg(target_os = "linux")]
use libsystemd::daemon::{self, NotifyState};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

pub const ESCALATE_AT: u32 = 6;

#[derive(Default)]
pub struct Watchdog {
    // We only need the leaky bucket. The naive miss counter is gone.
    leaky: Arc<AtomicU32>,
}

impl Watchdog {
    pub fn new() -> Self {
        Self {
            leaky: Arc::new(AtomicU32::new(0)),
        }
    }

    /// Called on a successful, on-time tick.
    pub fn kick(&self) -> u32 {
        // 1. Decay the leaky bucket by 1 (saturating at 0)
        let mut new_val = 0;
        let _ = self
            .leaky
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |cur| {
                new_val = cur.saturating_sub(1);
                Some(new_val)
            });

        // 2. Ping the outer OS guard (systemd)
        #[cfg(target_os = "linux")]
        let _ = daemon::notify(false, &[NotifyState::Watchdog]);

        new_val
    }

    /// Called when the loop overruns its deadline.
    pub fn miss(&self) -> u32 {
        // Add 2 to the leaky counter (saturating)
        let mut new_val = 0;
        let _ = self
            .leaky
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |cur| {
                new_val = cur.saturating_add(2);
                Some(new_val)
            });

        new_val
    }

    /// Check if we need to ESTOP
    pub fn should_escalate(&self) -> bool {
        self.leaky.load(Ordering::Relaxed) >= ESCALATE_AT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leaky_miss_kick_behaviour() {
        let wd = Watchdog::new();

        // 3 consecutive misses adds 2 each time -> 6
        assert_eq!(wd.miss(), 2);
        assert!(!wd.should_escalate());

        assert_eq!(wd.miss(), 4);
        assert!(!wd.should_escalate());

        assert_eq!(wd.miss(), 6);
        assert!(
            wd.should_escalate(),
            "3 consecutive misses should trigger escalation"
        );

        // Decay via kicks
        assert_eq!(wd.kick(), 5);
        assert_eq!(wd.kick(), 4);
    }

    #[test]
    fn leaky_never_underflows() {
        let wd = Watchdog::new();
        for _ in 0..10 {
            wd.kick();
        }
        assert_eq!(wd.leaky.load(Ordering::Relaxed), 0);
    }
}
