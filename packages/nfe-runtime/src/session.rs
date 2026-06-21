use nfe_core::telemetry::TelemetryEvent;

use crate::telemetry_bus::TelemetryBus;

pub fn publish_event(bus: &TelemetryBus, event: TelemetryEvent) {
    bus.publish(event);
}

pub fn publish_events(bus: &TelemetryBus, events: impl IntoIterator<Item = TelemetryEvent>) {
    for event in events {
        publish_event(bus, event);
    }
}
