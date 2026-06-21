use std::io;
use std::net::{SocketAddr, UdpSocket};

use nfe_core::telemetry::StartGateTelemetry;
use tracing::warn;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartGateMode {
    Live,
    Sim,
    Replay,
}

impl StartGateMode {
    fn as_str(self) -> &'static str {
        match self {
            StartGateMode::Live => "live",
            StartGateMode::Sim => "sim",
            StartGateMode::Replay => "replay",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartGateState {
    Closed,
    Open,
}

impl StartGateState {
    fn as_str(self) -> &'static str {
        match self {
            StartGateState::Closed => "closed",
            StartGateState::Open => "open",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartGatePolicy {
    UdpArm,
    Delay,
    AlwaysOpen,
}

#[derive(Clone, Debug)]
pub struct StartGateConfig {
    pub policy: StartGatePolicy,
    pub sim_start_delay_ms: u64,
    pub replay_start_delay_ms: u64,
    pub require_first_pipeline_cycle: bool,
    pub force_arm: bool,
    pub allow_live_force_arm: bool,
}

impl StartGateConfig {
    pub fn for_mode(mode: StartGateMode) -> Self {
        match mode {
            StartGateMode::Live => Self {
                policy: StartGatePolicy::UdpArm,
                sim_start_delay_ms: 100,
                replay_start_delay_ms: 0,
                require_first_pipeline_cycle: true,
                force_arm: false,
                allow_live_force_arm: false,
            },
            StartGateMode::Sim => Self {
                policy: StartGatePolicy::Delay,
                sim_start_delay_ms: 100,
                replay_start_delay_ms: 0,
                require_first_pipeline_cycle: true,
                force_arm: false,
                allow_live_force_arm: true,
            },
            StartGateMode::Replay => Self {
                policy: StartGatePolicy::AlwaysOpen,
                sim_start_delay_ms: 100,
                replay_start_delay_ms: 0,
                require_first_pipeline_cycle: true,
                force_arm: false,
                allow_live_force_arm: true,
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmSignal {
    None,
    Arm,
    Disarm,
}

pub trait ArmSignalSource: Send {
    fn poll(&mut self) -> anyhow::Result<ArmSignal>;
}

#[derive(Clone, Debug, Default)]
pub struct ArmSignalConfig {
    pub udp_bind: Option<String>,
    pub udp_token: Option<String>,
    pub gpio_enabled: bool,
    pub gpio_pin: Option<u8>,
}

struct CombinedArmSignalSource {
    sources: Vec<Box<dyn ArmSignalSource>>,
}

impl CombinedArmSignalSource {
    fn new(sources: Vec<Box<dyn ArmSignalSource>>) -> Self {
        Self { sources }
    }
}

impl ArmSignalSource for CombinedArmSignalSource {
    fn poll(&mut self) -> anyhow::Result<ArmSignal> {
        for source in &mut self.sources {
            let signal = source.poll()?;
            if signal != ArmSignal::None {
                return Ok(signal);
            }
        }
        Ok(ArmSignal::None)
    }
}

pub struct NoopArmSignalSource;

impl ArmSignalSource for NoopArmSignalSource {
    fn poll(&mut self) -> anyhow::Result<ArmSignal> {
        Ok(ArmSignal::None)
    }
}

pub struct UdpArmSignalSource {
    socket: UdpSocket,
    token: String,
    buf: [u8; 512],
}

impl UdpArmSignalSource {
    pub fn bind(
        addr: impl std::net::ToSocketAddrs,
        token: impl Into<String>,
    ) -> anyhow::Result<Self> {
        let socket = UdpSocket::bind(addr)?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            socket,
            token: token.into(),
            buf: [0; 512],
        })
    }

    fn parse(&self, data: &[u8], from: SocketAddr) -> ArmSignal {
        let Ok(text) = std::str::from_utf8(data).map(str::trim) else {
            warn!(%from, "arm udp: ignoring non-utf8 payload");
            return ArmSignal::None;
        };
        let mut parts = text.split_whitespace();
        if parts.next() != Some("NFE_ARM") {
            warn!(%from, "arm udp: ignoring payload with invalid prefix");
            return ArmSignal::None;
        }
        let Some(token) = parts.next() else {
            warn!(%from, "arm udp: ignoring payload without token");
            return ArmSignal::None;
        };
        if token != self.token {
            warn!(%from, "arm udp: ignoring payload with invalid token");
            return ArmSignal::None;
        }
        match parts.next() {
            Some("arm") => {
                warn!(%from, "arm udp: ARM accepted");
                ArmSignal::Arm
            }
            Some("disarm") => {
                warn!(%from, "arm udp: DISARM accepted");
                ArmSignal::Disarm
            }
            _ => {
                warn!(%from, "arm udp: ignoring payload with invalid command");
                ArmSignal::None
            }
        }
    }
}

impl ArmSignalSource for UdpArmSignalSource {
    fn poll(&mut self) -> anyhow::Result<ArmSignal> {
        let mut latest = ArmSignal::None;
        loop {
            match self.socket.recv_from(&mut self.buf) {
                Ok((n, from)) => {
                    let signal = self.parse(&self.buf[..n], from);
                    if signal != ArmSignal::None {
                        latest = signal;
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(latest),
                Err(e) => return Err(e.into()),
            }
        }
    }
}

#[cfg(target_os = "linux")]
pub struct GpioArmSignalSource {
    pin: rppal::gpio::InputPin,
}

#[cfg(target_os = "linux")]
impl GpioArmSignalSource {
    const DEBOUNCE_MS: u64 = 50;

    pub fn new(pin: u8) -> anyhow::Result<Self> {
        use rppal::gpio::{Gpio, Trigger};

        let mut pin = Gpio::new()?.get(pin)?.into_input_pullup();
        pin.set_interrupt(
            Trigger::Both,
            Some(std::time::Duration::from_millis(Self::DEBOUNCE_MS)),
        )?;
        Ok(Self { pin })
    }

    fn signal_from_event(event: rppal::gpio::Event) -> ArmSignal {
        match event.trigger {
            rppal::gpio::Trigger::FallingEdge => ArmSignal::Arm,
            rppal::gpio::Trigger::RisingEdge => ArmSignal::Disarm,
            _ => ArmSignal::None,
        }
    }
}

#[cfg(target_os = "linux")]
impl ArmSignalSource for GpioArmSignalSource {
    fn poll(&mut self) -> anyhow::Result<ArmSignal> {
        match self
            .pin
            .poll_interrupt(false, Some(std::time::Duration::ZERO))?
        {
            Some(event) => {
                let signal = Self::signal_from_event(event);
                match signal {
                    ArmSignal::Arm => warn!("arm gpio: ARM accepted"),
                    ArmSignal::Disarm => warn!("arm gpio: DISARM accepted"),
                    ArmSignal::None => {}
                }
                Ok(signal)
            }
            None => Ok(ArmSignal::None),
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub struct GpioArmSignalSource;

#[cfg(not(target_os = "linux"))]
impl GpioArmSignalSource {
    pub fn new(_pin: u8) -> anyhow::Result<Self> {
        anyhow::bail!("GPIO arm source requires a Linux target")
    }
}

#[cfg(not(target_os = "linux"))]
impl ArmSignalSource for GpioArmSignalSource {
    fn poll(&mut self) -> anyhow::Result<ArmSignal> {
        Ok(ArmSignal::None)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StartGateInput {
    pub timestamp_us: u64,
    pub start_line_crossed: bool,
    pub arm_signal: ArmSignal,
    pub pipeline_cycle_completed: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct StartGateDecision {
    pub state: StartGateState,
    pub allow_actuation: bool,
}

pub struct StartGateRuntime {
    gate: StartGate,
    arm_source: Box<dyn ArmSignalSource>,
}

impl StartGateRuntime {
    pub fn new(
        mode: StartGateMode,
        config: StartGateConfig,
        arm_config: ArmSignalConfig,
    ) -> anyhow::Result<Self> {
        let mut sources: Vec<Box<dyn ArmSignalSource>> = Vec::new();
        if matches!(
            (mode, config.policy),
            (StartGateMode::Live, StartGatePolicy::UdpArm)
        ) {
            if let Some(bind) = arm_config.udp_bind {
                sources.push(Box::new(UdpArmSignalSource::bind(
                    bind,
                    arm_config.udp_token.unwrap_or_default(),
                )?));
            }
        }
        if arm_config.gpio_enabled {
            let pin = arm_config
                .gpio_pin
                .ok_or_else(|| anyhow::anyhow!("GPIO arm source enabled without gpio_pin"))?;
            sources.push(Box::new(GpioArmSignalSource::new(pin)?));
        }
        let arm_source: Box<dyn ArmSignalSource> = if sources.is_empty() {
            Box::new(NoopArmSignalSource)
        } else {
            Box::new(CombinedArmSignalSource::new(sources))
        };
        Ok(Self {
            gate: StartGate::new(mode, config),
            arm_source,
        })
    }

    pub fn observe_tick(
        &mut self,
        timestamp_us: u64,
        start_line_crossed: bool,
        pipeline_cycle_completed: bool,
    ) -> anyhow::Result<(StartGateDecision, Option<StartGateTelemetry>)> {
        let arm_signal = self.arm_source.poll()?;
        Ok(self.gate.observe_tick(StartGateInput {
            timestamp_us,
            start_line_crossed,
            arm_signal,
            pipeline_cycle_completed,
        }))
    }
}

pub struct StartGate {
    mode: StartGateMode,
    config: StartGateConfig,
    state: StartGateState,
    created_at_us: Option<u64>,
    opened_at_us: Option<u64>,
    last_published_state: Option<StartGateState>,
    last_published_reason: Option<&'static str>,
}

impl StartGate {
    pub fn new(mode: StartGateMode, config: StartGateConfig) -> Self {
        let state = if matches!(config.policy, StartGatePolicy::AlwaysOpen) {
            StartGateState::Open
        } else {
            StartGateState::Closed
        };
        Self {
            mode,
            config,
            state,
            created_at_us: None,
            opened_at_us: None,
            last_published_state: None,
            last_published_reason: None,
        }
    }

    pub fn state(&self) -> StartGateState {
        self.state
    }

    pub fn is_open(&self) -> bool {
        self.state == StartGateState::Open
    }

    pub fn observe_tick(
        &mut self,
        input: StartGateInput,
    ) -> (StartGateDecision, Option<StartGateTelemetry>) {
        let _ = input.start_line_crossed;
        let created_at = *self.created_at_us.get_or_insert(input.timestamp_us);
        let mut reason = "waiting";
        let mut disarmed_this_tick = false;

        match input.arm_signal {
            ArmSignal::Disarm => {
                self.state = StartGateState::Closed;
                self.opened_at_us = None;
                reason = "disarmed";
                disarmed_this_tick = true;
            }
            ArmSignal::Arm => {
                if !self.config.require_first_pipeline_cycle || input.pipeline_cycle_completed {
                    self.open(input.timestamp_us);
                    reason = "armed";
                } else {
                    reason = "waiting_for_first_pipeline_cycle";
                }
            }
            ArmSignal::None => {}
        }

        if self.state == StartGateState::Closed && !disarmed_this_tick {
            if self.config.force_arm
                && (self.mode != StartGateMode::Live || self.config.allow_live_force_arm)
            {
                if !self.config.require_first_pipeline_cycle || input.pipeline_cycle_completed {
                    self.open(input.timestamp_us);
                    reason = "force_armed";
                } else {
                    reason = "waiting_for_first_pipeline_cycle";
                }
            } else if self.config.force_arm && self.mode == StartGateMode::Live {
                reason = "live_force_arm_rejected";
            } else if matches!(self.config.policy, StartGatePolicy::Delay) {
                let delay_ms = match self.mode {
                    StartGateMode::Sim => self.config.sim_start_delay_ms,
                    StartGateMode::Replay => self.config.replay_start_delay_ms,
                    StartGateMode::Live => 0,
                };
                let delay_elapsed =
                    input.timestamp_us.saturating_sub(created_at) >= delay_ms.saturating_mul(1_000);
                if delay_elapsed
                    && (!self.config.require_first_pipeline_cycle || input.pipeline_cycle_completed)
                {
                    self.open(input.timestamp_us);
                    reason = "delay_elapsed";
                } else if !input.pipeline_cycle_completed
                    && self.config.require_first_pipeline_cycle
                {
                    reason = "waiting_for_first_pipeline_cycle";
                } else {
                    reason = "waiting_for_delay";
                }
            } else if matches!(self.config.policy, StartGatePolicy::AlwaysOpen) {
                self.open(input.timestamp_us);
                reason = "always_open";
            } else if matches!(self.config.policy, StartGatePolicy::UdpArm) {
                reason = "waiting_for_udp_arm";
            }
        } else if reason == "waiting" {
            reason = "open";
        }

        let decision = StartGateDecision {
            state: self.state,
            allow_actuation: self.is_open(),
        };
        let telemetry =
            self.transition_telemetry(input.timestamp_us, reason, decision.allow_actuation);
        (decision, telemetry)
    }

    pub fn arm(&mut self, timestamp_us: u64) {
        self.open(timestamp_us);
    }

    pub fn disarm(&mut self) {
        self.state = StartGateState::Closed;
        self.opened_at_us = None;
    }

    pub fn reset(&mut self) {
        self.state = if matches!(self.config.policy, StartGatePolicy::AlwaysOpen) {
            StartGateState::Open
        } else {
            StartGateState::Closed
        };
        self.created_at_us = None;
        self.opened_at_us = None;
        self.last_published_state = None;
        self.last_published_reason = None;
    }

    fn open(&mut self, timestamp_us: u64) {
        self.state = StartGateState::Open;
        self.opened_at_us.get_or_insert(timestamp_us);
    }

    fn transition_telemetry(
        &mut self,
        timestamp_us: u64,
        reason: &'static str,
        allow_actuation: bool,
    ) -> Option<StartGateTelemetry> {
        if self.last_published_state == Some(self.state)
            && self.last_published_reason == Some(reason)
        {
            return None;
        }
        self.last_published_state = Some(self.state);
        self.last_published_reason = Some(reason);
        Some(StartGateTelemetry {
            timestamp_us,
            state: self.state.as_str().to_string(),
            allow_actuation,
            reason: reason.to_string(),
            mode: self.mode.as_str().to_string(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_delay_requires_first_cycle_even_with_zero_delay() {
        let mut cfg = StartGateConfig::for_mode(StartGateMode::Sim);
        cfg.sim_start_delay_ms = 0;
        let mut gate = StartGate::new(StartGateMode::Sim, cfg);
        let (d, _) = gate.observe_tick(StartGateInput {
            timestamp_us: 0,
            start_line_crossed: false,
            arm_signal: ArmSignal::None,
            pipeline_cycle_completed: false,
        });
        assert!(!d.allow_actuation);
        let (d, _) = gate.observe_tick(StartGateInput {
            timestamp_us: 10_000,
            start_line_crossed: false,
            arm_signal: ArmSignal::None,
            pipeline_cycle_completed: true,
        });
        assert!(d.allow_actuation);
    }

    #[test]
    fn udp_arm_opens_live_gate() {
        let cfg = StartGateConfig::for_mode(StartGateMode::Live);
        let mut gate = StartGate::new(StartGateMode::Live, cfg);
        let (d, _) = gate.observe_tick(StartGateInput {
            timestamp_us: 1,
            start_line_crossed: false,
            arm_signal: ArmSignal::Arm,
            pipeline_cycle_completed: true,
        });
        assert!(d.allow_actuation);
    }

    #[test]
    fn disarm_recloses_gate() {
        let cfg = StartGateConfig::for_mode(StartGateMode::Replay);
        let mut gate = StartGate::new(StartGateMode::Replay, cfg);
        assert!(gate.is_open());
        let (d, _) = gate.observe_tick(StartGateInput {
            timestamp_us: 1,
            start_line_crossed: false,
            arm_signal: ArmSignal::Disarm,
            pipeline_cycle_completed: true,
        });
        assert!(!d.allow_actuation);
    }
}
