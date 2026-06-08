/// Peremt le controle de l'ESC et du servo
///
/// TODO: dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2

use rppal::gpio::Gpio;
use rppal::pwm::{Channel, Polarity, Pwm};
use tracing::warn;


const PWM_FREQ_HZ:    f64 = 50.0;
const PWM_PERIOD_US:  f64 = 1_000_000.0 / PWM_FREQ_HZ;   // 20_000 µs

const PULSE_MIN_US:     f64 = 1000.0; // unit en microseconds
const PULSE_NEUTRAL_US: f64 = 1500.0;
const PULSE_MAX_US:     f64 = 2000.0;
const PULSE_RANGE_US:   f64 = 500.0;   // neutral ± this = full deflection

/// Angle maximal du servo (±0.5 rad ≈ ±28.6°)
pub const SERVO_MAX_RAD: f32 = 0.5;

/// Fraction maximal du throttle (1.0 = full forward)
pub const THROTTLE_MAX: f32 = 1.0;


pub struct Actuate {
    esc:   Pwm,
    servo: Pwm,
}

impl Actuate {
    pub fn new(_gpio: &Gpio) -> anyhow::Result<Self> {
        let period = std::time::Duration::from_micros(PWM_PERIOD_US as u64);
        let neutral = std::time::Duration::from_micros(PULSE_NEUTRAL_US as u64);

        let esc = Pwm::with_period(Channel::Pwm0, period, neutral, Polarity::Normal, true)?;
        let servo = Pwm::with_period(Channel::Pwm1, period, neutral, Polarity::Normal, true)?;

        Ok(Self { esc, servo })
    }

    /// `throttle`: -1.0 (full brake/reverse) … +1.0 (full forward)
    pub fn set_pwm_esc(&mut self, throttle: f32) {
        let t = throttle.clamp(-THROTTLE_MAX, THROTTLE_MAX) as f64;
        let pulse_us = PULSE_NEUTRAL_US + t * PULSE_RANGE_US;
        Self::write_pwm(&self.esc, pulse_us);
    }

    /// `angle_rad`: -SERVO_MAX_RAD (full right) … +SERVO_MAX_RAD (full left)
    pub fn set_pwm_servo(&mut self, angle_rad: f32) {
        let a = angle_rad.clamp(-SERVO_MAX_RAD, SERVO_MAX_RAD) as f64;
        let pulse_us = PULSE_NEUTRAL_US + (a / SERVO_MAX_RAD as f64) * PULSE_RANGE_US;
        Self::write_pwm(&self.servo, pulse_us);
    }

    /// Passert en neutral et roues de face lors d'un shutdown ou du watchdog miss
    pub fn safe_state(&mut self) {
        Self::write_pwm(&self.esc,   PULSE_NEUTRAL_US);
        Self::write_pwm(&self.servo, PULSE_NEUTRAL_US);
        warn!("actuate: safe_state — ESC neutral, servo centre");
    }

    fn write_pwm(pwm: &Pwm, pulse_us: f64) {
        let pulse = std::time::Duration::from_nanos((pulse_us * 1000.0) as u64);
        if let Err(e) = pwm.set_pulse_width(pulse) {
            warn!("actuate: PWM write error: {e}");
        }
    }
}

impl Drop for Actuate {
    fn drop(&mut self) {
        // Passer en neutral sur un drop (sorti du processus / panic)
        Self::write_pwm(&self.esc,   PULSE_NEUTRAL_US);
        Self::write_pwm(&self.servo, PULSE_NEUTRAL_US);
    }
}
