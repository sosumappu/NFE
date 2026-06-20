/// telemetry.rs — fan-out telemetry bus
///
/// The control loop is the single producer of structured telemetry. Multiple
/// consumers (MCAP recorder, Foxglove bridge) each
/// need a filtered subset of that data at different rates. Giving each consumer
/// a direct reference to the control loop would create tight coupling, and a
/// single shared channel would force all consumers to agree on one buffer size
/// and one drop policy.
use std::sync::{mpsc, Arc, Mutex};

use crate::hal::TimestampedFrame;
use crate::metrics::TickMetrics;
use crate::replay::recorder::ControlFrame;

// ── Event type ─────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub enum TelemetryEvent {
    Metrics(Arc<TickMetrics>),
    Sensor(TimestampedFrame),
    Control(ControlFrame),
}

// ── Bus ────────────────────────────────────────────────────────────────────

type Sender = mpsc::SyncSender<TelemetryEvent>;

/// Producer handle. Cheap to clone — all clones share the same subscriber
/// list. The control loop holds one clone; modes/live.rs holds another for
/// wiring subscribers before the loop starts.
///
/// The inner Mutex is held only during publish() (to iterate the Vec) and
/// subscribe() (to push a new sender). At 100 Hz with 2-3 subscribers the
/// lock is uncontended and acquisition costs ~5 ns on aarch64. If that ever
/// shows up in a profile, replace Vec<Sender> with ArcSwap<Vec<Sender>> so
/// publish() is entirely lock-free.
#[derive(Clone)]
pub struct TelemetryBus {
    senders: Arc<Mutex<Vec<Sender>>>,
}

impl TelemetryBus {
    pub fn new() -> Self {
        Self {
            senders: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Register a new subscriber and return its receiving end. Call this
    /// before handing the bus to the control loop — subscribers registered
    /// after publish() starts will miss earlier events.
    ///
    /// `capacity` is the channel bound in number of events. Tune it to the
    /// consumer's throughput:
    ///   MCAP recorder  — 2048: disk I/O is bursty; a large buffer absorbs
    ///                    write latency spikes without dropping sensor frames.
    ///   Foxglove bridge — 64: the bridge polls every 50 ms and only needs
    ///                    the latest metrics; a small buffer is fine and
    ///                    limits memory if the bridge stalls.
    ///   Prometheus      — 128: scrape interval is 15 s, but metrics arrive
    ///                    at 100 Hz so we buffer a few seconds of headroom.
    pub fn subscribe(&self, capacity: usize) -> mpsc::Receiver<TelemetryEvent> {
        let (tx, rx) = mpsc::sync_channel(capacity);
        self.senders.lock().unwrap().push(tx);
        rx
    }

    /// Publish one event to every registered subscriber. Non-blocking: if a
    /// subscriber's channel is full, try_send returns Err and we discard the
    /// frame for that subscriber only. This keeps the 100 Hz control loop
    /// deadline safe regardless of how slow any individual consumer is.
    ///
    /// Disconnected senders (subscriber thread exited) are pruned here so the
    /// Vec doesn't grow unboundedly in long-running sessions where consumers
    /// are stopped and restarted.
    #[inline]
    pub fn publish(&self, event: TelemetryEvent) {
        let mut senders = self.senders.lock().unwrap();
        if senders.is_empty() {
            return;
        }
        senders.retain(|tx| {
            // try_send returns Err(Disconnected) when the receiver is gone —
            // prune those eagerly so we don't iterate dead channels forever.
            match tx.try_send(event.clone()) {
                Ok(()) => true,
                Err(mpsc::TrySendError::Full(_)) => true, // slow but alive
                Err(mpsc::TrySendError::Disconnected(_)) => false, // dead — remove
            }
        });
    }

    /// True when no subscribers are registered. The control loop uses this to
    /// skip Arc::new(m) allocation entirely when nothing is listening — useful
    /// in replay and sim modes where the bus is None anyway, but also during
    /// startup before any subscriber has called subscribe().
    pub fn is_empty(&self) -> bool {
        self.senders.lock().unwrap().is_empty()
    }
}

impl Default for TelemetryBus {
    fn default() -> Self {
        Self::new()
    }
}
