//! This module contains an archive of (some of) the code that has been used to
//! generate the tables used for simulation.

#![allow(dead_code)]

use rlbot;
use std::error::Error;

/// Achieve maximum boosting speed, then immediately release the throttle and
/// boost and coast until stopping.
pub fn throttle(rlbot: &rlbot::RLBot, time: f32) -> Result<bool, Box<Error>> {
    if time < 2.0 {
        let input = Default::default();
        rlbot.update_player_input(input, 0)?;
        Ok(true)
    } else if time < 5.0 {
        let input = rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            ..Default::default()
        };
        rlbot.update_player_input(input, 0)?;
        Ok(true)
    } else {
        Ok(false)
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

/// Aerial while trying (very naively) to achieve a pitch of 60°.
pub fn aerial_60deg(
    rlbot: &rlbot::RLBot,
    time: f32,
    packet: &rlbot::ffi::LiveDataPacket,
) -> Result<bool, Box<Error>> {
    if time < 1.0 {
        let input = Default::default();
        rlbot.update_player_input(input, 0)?;
    } else if time < 5.0 {
        let pitch = (60.0_f32.to_radians() - packet.GameCars[0].Physics.Rotation.Pitch) / 2.0;
        let input = rlbot::ffi::PlayerInput {
            Pitch: pitch.max(-1.0).min(1.0),
            Jump: true,
            Boost: time >= 1.25,
            ..Default::default()
        };
        rlbot.update_player_input(input, 0)?;
    } else {
        return Ok(false);
    }
    Ok(true)
}
