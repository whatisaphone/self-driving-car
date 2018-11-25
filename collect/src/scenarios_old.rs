//! This module contains an archive of (some of) the code that has been used to
//! generate the tables used for simulation.

#![allow(dead_code)]

use rlbot;
use std::error::Error;

/// Achieve maximum boosting speed, then immediately release the throttle and
/// boost and coast until stopping.
pub fn throttle(
    time: f32,
    _packet: &rlbot::ffi::LiveDataPacket,
) -> Option<rlbot::ffi::PlayerInput> {
    if time < 2.0 {
        Some(Default::default())
    } else if time < 5.0 {
        Some(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            ..Default::default()
        })
    } else {
        None
    }
}

/// Achieve maximum boosting speed, then immediately release the throttle and
/// boost and coast until stopping.
pub fn coast(
    rlbot: &rlbot::RLBot,
    time: f32,
    _packet: &rlbot::ffi::LiveDataPacket,
) -> Result<bool, Box<Error>> {
    if time < 1.0 {
        let input = Default::default();
        rlbot.update_player_input(input, 0)?;
    } else if time < 3.0 {
        let input = rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Boost: true,
            ..Default::default()
        };
        rlbot.update_player_input(input, 0)?;
    } else if time < 8.0 {
        let input = Default::default();
        rlbot.update_player_input(input, 0)?;
    } else {
        return Ok(false);
    }
    Ok(true)
}
