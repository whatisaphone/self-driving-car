//! Various Rocket League constants.

/// The distance from the field center to the side wall.
///
/// This value was copied from https://github.com/RLBot/RLBot/wiki/Useful-Game-Values.
pub const FIELD_MAX_X: f32 = 4096.0;

/// The distance from the field center to the back wall.
///
/// This value was copied from https://github.com/RLBot/RLBot/wiki/Useful-Game-Values.
pub const FIELD_MAX_Y: f32 = 5120.0;

/// The distance from the ground (z = 0) to the ceiling.
///
/// Source: observed in-game
pub const FIELD_MAX_Z: f32 = 2046.68;

/// The z-coordinate of the crossbar.
///
/// This value was copied from https://github.com/RLBot/RLBot/wiki/Useful-Game-Values.
pub const CROSSBAR_Z: f32 = 642.775;

/// The absolute value of the x-coordinate of the goalposts.
///
/// This value was copied from https://github.com/RLBot/RLBot/wiki/Useful-Game-Values.
pub const GOALPOST_X: f32 = 892.755;

/// Source: https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
pub const GRAVITY: f32 = -650.0;

/// The radius of the ball.
///
/// This value was observed in data from `collect`.
pub const BALL_RADIUS: f32 = 91.24;

/// Source: https://discordapp.com/channels/348658686962696195/348659150793736193/525795583307415552
pub const BALL_MASS: f32 = 30.0;

/// Source: https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
pub const CAR_MASS: f32 = 180.0;

/// The z location of the Octane when sitting on the ground.
///
/// This value was observed in data from `collect`.
pub const OCTANE_NEUTRAL_Z: f32 = 17.01;

/// The constant frequency of RL's physics engine.
pub const PHYSICS_TICK_FREQ: f32 = 120.0;

/// The number of seconds between physics ticks.
pub const PHYSICS_DT: f32 = 1.0 / PHYSICS_TICK_FREQ;

/// The max speed a car can reach using only the throttle.
///
/// This value was observed in data from `collect`.
pub const CAR_NORMAL_SPEED: f32 = 1410.001;

/// The max speed a car can reach by boosting.
///
/// This value was observed in data from `collect`.
pub const CAR_MAX_SPEED: f32 = 2299.981;

/// Almost max speed. This is a placeholder for behaviors where some sort of
/// boost hysteresis would have been appropriate but I was too lazy to
/// implement it.
pub const CAR_ALMOST_MAX_SPEED: f32 = CAR_MAX_SPEED - 10.0;

/// Force applied on the first frame of a jump.
///
/// Source: RLBot magic_numbers.py
pub const CAR_JUMP_IMPULSE: f32 = 52_500.0;

/// Additional force applied during a jump while the button is held down.
///
/// Source: RLBot magic_numbers.py
pub const CAR_JUMP_FORCE: f32 = 262_500.0;

/// The amount of time additional force can be added during a jump while holding
/// the button down.
///
/// Source: RLBot magic_numbers.py
pub const CAR_JUMP_FORCE_TIME: f32 = 0.2;

/// The instantaneous velocity change on the first frame of a jump.
pub const CAR_JUMP_IMPULSE_SPEED: f32 = CAR_JUMP_IMPULSE / CAR_MASS;

/// The additional acceleration during a jump while holding the button down.
pub const CAR_JUMP_ACCEL: f32 = CAR_JUMP_FORCE / CAR_MASS;

/// Boost depletion per second.
///
/// This value was determined using data from `collect`.
pub const BOOST_DEPLETION: f32 = 100.0 / 3.0;

/// The velocity increase when dodging forward.
pub const DODGE_FORWARD_IMPULSE: f32 = 500.0;

/// The radius of the full boost pickup's cylindrical hitbox.
///
/// Source: https://youtu.be/xgfa-qZyInw?t=31
pub const BOOST_DOLLAR_RADIUS: f32 = 208.0;

/// The height of the full boost pickup's cylindrical hitbox.
///
/// Source: https://youtu.be/xgfa-qZyInw?t=31
pub const BOOST_DOLLAR_HEIGHT: f32 = 168.0;

/// The radius of the small boost pickup's cylindrical hitbox.
///
/// Source: https://youtu.be/xgfa-qZyInw?t=31
pub const BOOST_PENNY_RADIUS: f32 = 144.0;

/// The height of the small boost pickup's cylindrical hitbox.
///
/// Source: https://youtu.be/xgfa-qZyInw?t=31
pub const BOOST_PENNY_HEIGHT: f32 = 165.0;
