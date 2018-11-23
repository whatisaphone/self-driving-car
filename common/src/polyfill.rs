use std::time::Duration;

const MILLIS_PER_SEC: u64 = 1_000;

pub trait ExtendDuration {
    // Tracking issue: https://github.com/rust-lang/rust/issues/50202
    fn as_millis_polyfill(&self) -> u128;
}

impl ExtendDuration for Duration {
    fn as_millis_polyfill(&self) -> u128 {
        self.as_secs() as u128 * MILLIS_PER_SEC as u128 + self.subsec_millis() as u128
    }
}
