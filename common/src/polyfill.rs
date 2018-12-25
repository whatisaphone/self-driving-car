use std::time::Duration;

const MILLIS_PER_SEC: u64 = 1_000;

pub trait ExtendDuration {
    // Tracking issue: https://github.com/rust-lang/rust/issues/50202
    fn as_millis_polyfill(&self) -> u128;
}

impl ExtendDuration for Duration {
    fn as_millis_polyfill(&self) -> u128 {
        u128::from(self.as_secs()) * u128::from(MILLIS_PER_SEC) + u128::from(self.subsec_millis())
    }
}
