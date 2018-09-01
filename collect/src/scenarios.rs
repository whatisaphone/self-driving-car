use rlbot;
use std::error::Error;

pub fn coast(
    rlbot: &rlbot::RLBot,
    time: f32,
    _packet: &rlbot::LiveDataPacket,
) -> Result<bool, Box<Error>> {
    if time < 1.0 {
        let input = Default::default();
        rlbot.update_player_input(input, 0)?;
    } else if time < 3.0 {
        let input = rlbot::PlayerInput {
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
