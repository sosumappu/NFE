/// Debounces a physical start/finish-line crossing signal and emits rising
/// edges. The hardware adapter should provide a raw boolean from the laser gate
/// or orange-line detector; the supervisor only sees debounced edges.
#[derive(Clone, Debug)]
pub struct StartLineDebouncer {
    debounce_ticks: u32,
    stable_true: u32,
    latched: bool,
}

impl StartLineDebouncer {
    pub fn new(debounce_ticks: u32) -> Self {
        Self {
            debounce_ticks: debounce_ticks.max(1),
            stable_true: 0,
            latched: false,
        }
    }

    /// Returns true exactly once per physical crossing.
    pub fn update(&mut self, raw_crossed: bool) -> bool {
        if raw_crossed {
            self.stable_true = self.stable_true.saturating_add(1);
        } else {
            self.stable_true = 0;
            self.latched = false;
        }

        if self.stable_true >= self.debounce_ticks && !self.latched {
            self.latched = true;
            true
        } else {
            false
        }
    }
}

impl Default for StartLineDebouncer {
    fn default() -> Self {
        Self::new(2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emits_one_edge_per_crossing() {
        let mut d = StartLineDebouncer::new(2);
        assert!(!d.update(false));
        assert!(!d.update(true));
        assert!(d.update(true));
        assert!(!d.update(true));
        assert!(!d.update(false));
        assert!(!d.update(true));
        assert!(d.update(true));
    }
}
