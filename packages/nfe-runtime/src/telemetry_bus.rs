//! Runtime telemetry bus implementation.
//!
//! The bus is a non-blocking fan-out of pure `nfe_core::telemetry::TelemetryEvent`
//! values. Recorders, live streamers, and analysis tools are subscribers.

use std::sync::{mpsc, Arc, Mutex};

use nfe_core::telemetry::TelemetryEvent;

pub type TelemetryReceiver = mpsc::Receiver<TelemetryEvent>;
type Sender = mpsc::SyncSender<TelemetryEvent>;

#[derive(Clone, Default)]
pub struct TelemetryBus {
    senders: Arc<Mutex<Vec<Sender>>>,
}

impl TelemetryBus {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn subscribe(&self, capacity: usize) -> TelemetryReceiver {
        let (tx, rx) = mpsc::sync_channel(capacity);
        self.senders.lock().unwrap().push(tx);
        rx
    }

    #[inline]
    pub fn publish(&self, event: TelemetryEvent) {
        let mut senders = self.senders.lock().unwrap();
        if senders.is_empty() {
            return;
        }
        senders.retain(|tx| match tx.try_send(event.clone()) {
            Ok(()) => true,
            Err(mpsc::TrySendError::Full(_)) => true,
            Err(mpsc::TrySendError::Disconnected(_)) => false,
        });
    }

    pub fn is_empty(&self) -> bool {
        self.senders.lock().unwrap().is_empty()
    }
}

/// Generic recorder/sink abstraction. Concrete sinks choose storage/transport
/// format (MCAP, JSONL, network, etc.) while consuming the same bus events.
pub trait TelemetrySink: Send + 'static {
    fn finish(self);
}
