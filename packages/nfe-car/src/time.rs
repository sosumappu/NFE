use std::sync::LazyLock;
use std::time::Instant;

pub static MONO_START: LazyLock<Instant> = LazyLock::new(Instant::now);

#[inline]
pub fn monotonic_us() -> u64 {
    Instant::now().duration_since(*MONO_START).as_micros() as u64
}
