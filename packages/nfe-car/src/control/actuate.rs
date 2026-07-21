/// control/actuate.rs — Actuator implementations
///
/// Three concrete types:
///
///   RealActuator      — rppal hardware PWM (Channel 0 = ESC, Channel 1 = servo)
///   DryRunActuator    — no-op, used when PWM hardware is absent
///   LoggingActuator   — decorator: wraps any ActuatorSink, traces every command
///
/// The `ActuatorFactory::build()` associated function probes for PWM hardware
/// at startup and returns the right concrete type, already wrapped in the
/// logging decorator. The control loop only ever sees `Box<dyn ActuatorSink>`.
///
/// This removes all `if let Some(act)` guards from the control loop: the
/// factory guarantees that a valid (possibly no-op) actuator is always present.
#[cfg(target_os = "linux")]
use anyhow::Context;
use anyhow::Result;
use tracing::{info, warn};

use crate::hal::ActuatorSink;

// ── Shared PWM constants ───────────────────────────────────────────────────

#[cfg(target_os = "linux")]
const PWM_FREQ_HZ: f64 = 50.0;
#[cfg(target_os = "linux")]
const PWM_PERIOD_US: f64 = 1_000_000.0 / PWM_FREQ_HZ; // 20 000 µs
#[cfg(target_os = "linux")]
const PULSE_NEUTRAL_US: f64 = 1_500.0;
#[cfg(target_os = "linux")]
const PULSE_RANGE_US: f64 = 500.0; // neutral ± this = full deflection
#[cfg(target_os = "linux")]
const DEADBAND_US: f64 = 25.0; // 5% of 500µs

pub const SERVO_MAX_RAD: f32 = 0.7; // ≈ ±40.2°
pub const THROTTLE_MAX: f32 = 1.0;

#[cfg(target_os = "linux")]
#[inline]
fn throttle_to_pulse(throttle: f32) -> f64 {
    let t = throttle.clamp(-1.0, 1.0) as f64;
    if t == 0.0 {
        return PULSE_NEUTRAL_US;
    }
    let raw = PULSE_NEUTRAL_US + t * PULSE_RANGE_US;
    // Push values that land inside the deadband outward past the deadband edge
    if t > 0.0 {
        raw.max(PULSE_NEUTRAL_US + DEADBAND_US + 1.0)
    } else {
        raw.min(PULSE_NEUTRAL_US - DEADBAND_US - 1.0)
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  RealActuator
// ══════════════════════════════════════════════════════════════════════════

#[cfg(target_os = "linux")]
mod real {
    use super::*;
    use rppal::pwm::{Polarity, Pwm};

    // The Raspberry Pi 5 uses the RP1 co-processor for GPIO/PWM. rppal ≥0.22
    // maps Channel::Pwm0/Pwm1 to pwmchip2 for Pi 5, but the RP1 PWM block
    // is registered as pwmchip0 in sysfs on this kernel build. The overlay
    // dtoverlay=pwm-2chan pin=18 pin2=19 maps:
    //   GPIO18 → PWM0_CHAN2 → pwmchip0 channel 2  (ESC)
    //   GPIO19 → PWM0_CHAN3 → pwmchip0 channel 3  (servo)
    // Use with_pwmchip to bypass the model-detection that yields the wrong chip.
    const ESC_CHIP: u8 = 0;
    const ESC_CHAN: u8 = 2;
    const SERVO_CHIP: u8 = 0;
    const SERVO_CHAN: u8 = 3;

    pub struct RealActuator {
        esc: Pwm,
        servo: Pwm,
    }

    impl RealActuator {
        pub fn try_new() -> Result<Self> {
            let period = std::time::Duration::from_micros(PWM_PERIOD_US as u64);
            let neutral = std::time::Duration::from_micros(PULSE_NEUTRAL_US as u64);

            let esc = {
                let pwm = Pwm::with_pwmchip(ESC_CHIP, ESC_CHAN)
                    .context("ESC (GPIO18) init failed — is dtoverlay=pwm-2chan in config.txt? Is /sys/class/pwm/pwmchip0 group-writable?")?;
                let _ = pwm.set_pulse_width(std::time::Duration::ZERO);
                pwm.set_period(period).context("ESC set_period")?;
                pwm.set_pulse_width(neutral).context("ESC set_pulse_width")?;
                pwm.set_polarity(Polarity::Normal).context("ESC set_polarity")?;
                pwm.enable().context("ESC enable")?;
                pwm
            };
            let servo = {
                let pwm = Pwm::with_pwmchip(SERVO_CHIP, SERVO_CHAN)
                    .context("servo (GPIO19) init failed")?;
                let _ = pwm.set_pulse_width(std::time::Duration::ZERO);
                pwm.set_period(period).context("servo set_period")?;
                pwm.set_pulse_width(neutral).context("servo set_pulse_width")?;
                pwm.set_polarity(Polarity::Normal).context("servo set_polarity")?;
                pwm.enable().context("servo enable")?;
                pwm
            };

            Ok(Self { esc, servo })
        }

        fn write(pwm: &Pwm, pulse_us: f64) -> Result<()> {
            let pulse = std::time::Duration::from_nanos((pulse_us * 1_000.0) as u64);
            pwm.set_pulse_width(pulse).context("PWM write failed")
        }
    }

    impl ActuatorSink for RealActuator {
        fn set_throttle(&mut self, throttle: f32) -> Result<()> {
            Self::write(&self.esc, throttle_to_pulse(throttle))
        }

        fn set_steering(&mut self, angle_rad: f32) -> Result<()> {
            let a = angle_rad.clamp(-SERVO_MAX_RAD, SERVO_MAX_RAD) as f64;
            Self::write(
                &self.servo,
                PULSE_NEUTRAL_US + (a / SERVO_MAX_RAD as f64) * PULSE_RANGE_US,
            )
        }

        fn safe_state(&mut self) -> Result<()> {
            Self::write(&self.esc, PULSE_NEUTRAL_US)?;
            Self::write(&self.servo, PULSE_NEUTRAL_US)
        }

        fn label(&self) -> &'static str {
            "real"
        }
    }

    impl Drop for RealActuator {
        fn drop(&mut self) {
            // Best-effort neutral on drop (process exit / panic)
            let _ = Self::write(&self.esc, PULSE_NEUTRAL_US);
            let _ = Self::write(&self.servo, PULSE_NEUTRAL_US);
        }
    }

    pub fn try_build_real() -> Result<Box<dyn ActuatorSink>> {
        Ok(Box::new(RealActuator::try_new()?))
    }
}

#[cfg(not(target_os = "linux"))]
mod real {
    use super::*;
    pub fn try_build_real() -> Result<Box<dyn ActuatorSink>> {
        anyhow::bail!("RealActuator is only available on Linux")
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  DryRunActuator
// ══════════════════════════════════════════════════════════════════════════

/// No-op actuator. Used automatically when hardware is unavailable.
/// The LoggingActuator decorator handles all output — this type stays silent.
pub struct DryRunActuator;

impl ActuatorSink for DryRunActuator {
    fn set_throttle(&mut self, _throttle: f32) -> Result<()> {
        Ok(())
    }
    fn set_steering(&mut self, _angle_rad: f32) -> Result<()> {
        Ok(())
    }
    fn safe_state(&mut self) -> Result<()> {
        Ok(())
    }
    fn label(&self) -> &'static str {
        "dry-run"
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  LoggingActuator — decorator
// ══════════════════════════════════════════════════════════════════════════

/// Wraps any `ActuatorSink` and emits a tracing event on every command.
/// Log level is configurable so you can silence it in hot production loops
/// while keeping it loud during development.
pub struct LoggingActuator {
    inner: Box<dyn ActuatorSink>,
    log_every_n: u32, // emit a log every N calls (1 = always)
    call_count: u32,
}

impl LoggingActuator {
    pub fn new(inner: Box<dyn ActuatorSink>, log_every_n: u32) -> Self {
        Self {
            inner,
            log_every_n,
            call_count: 0,
        }
    }

    fn should_log(&mut self) -> bool {
        self.call_count += 1;
        self.call_count.is_multiple_of(self.log_every_n)
    }
}

impl ActuatorSink for LoggingActuator {
    fn set_throttle(&mut self, throttle: f32) -> Result<()> {
        if self.should_log() {
            tracing::debug!(
                actuator = self.inner.label(),
                throttle = format!("{throttle:.4}"),
                "set_throttle"
            );
        }
        self.inner.set_throttle(throttle)
    }

    fn set_steering(&mut self, angle_rad: f32) -> Result<()> {
        if self.should_log() {
            tracing::debug!(
                actuator = self.inner.label(),
                angle_rad = format!("{:.2}", angle_rad),
                "set_steering"
            );
        }
        self.inner.set_steering(angle_rad)
    }

    fn safe_state(&mut self) -> Result<()> {
        warn!(actuator = self.inner.label(), "safe_state engaged");
        self.inner.safe_state()
    }

    fn label(&self) -> &'static str {
        self.inner.label()
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  ActuatorFactory
// ══════════════════════════════════════════════════════════════════════════

pub struct ActuatorFactory;

impl ActuatorFactory {
    /// Probe for PWM hardware. Falls back to DryRunActuator transparently.
    /// Always returns a `LoggingActuator` decorator around the chosen impl.
    ///
    /// `log_every_n`: emit a tracing log every N actuator calls.
    /// Use 1 during dev (every call), 10 in production (100 Hz → 10 Hz logs).
    pub fn build(log_every_n: u32) -> Box<dyn ActuatorSink> {
        let inner: Box<dyn ActuatorSink> = match real::try_build_real() {
            Ok(a) => {
                info!("actuator: PWM hardware detected — using RealActuator");
                a
            }
            Err(e) => {
                warn!("actuator: PWM hardware not available ({e:#}) — using DryRunActuator");
                Box::new(DryRunActuator)
            }
        };

        Box::new(LoggingActuator::new(inner, log_every_n))
    }
}
