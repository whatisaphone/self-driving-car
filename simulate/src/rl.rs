//! Various Rocket League constants.

/// Boost depletion per second.
///
/// This value was determined using data from `collect`.
pub const BOOST_DEPLETION: f32 = 100.0 / 3.0;

/// The max speed a car can reach using only the throttle.
///
/// This value was observed in data from `collect`.
pub const CAR_NORMAL_SPEED: f32 = 1410.0;

/// The max speed a car can reach by boosting.
///
/// This value was observed in data from `collect`.
pub const CAR_MAX_SPEED: f32 = 2299.98;
