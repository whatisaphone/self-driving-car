#![cfg_attr(feature = "strict", deny(warnings))]

extern crate csv;
extern crate flatbuffers;
extern crate nalgebra;
extern crate rlbot;

use collector2::Collector;
use game_state::{
    DesiredBallState, DesiredCarState, DesiredGameState, DesiredPhysics, RotatorPartial,
    Vector3Partial,
};
use rlbot_ext::get_packet_and_inject_rigid_body_tick;
use std::{error::Error, f32::consts::PI, fs::File, thread::sleep, time::Duration};

mod collector2;
mod game_state;
mod rlbot_ext;
mod scenarios;
mod utils;

pub fn main() -> Result<(), Box<Error>> {
    let rlbot = rlbot::init()?;
    start_match(&rlbot)?;

    let mut packets = rlbot.packeteer();

    // Wait for RoundActive
    while !packets.next()?.GameInfo.RoundActive {}

    // Zero out our input, just to be safe
    rlbot.update_player_input(Default::default(), 0)?;

    let initial_state = DesiredGameState {
        ball_state: Some(DesiredBallState {
            physics: Some(DesiredPhysics {
                location: Some(Vector3Partial::new(2000.0, 0.0, 0.0)),
                rotation: Some(RotatorPartial::new(0.0, 0.0, 0.0)),
                velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
                angular_velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
            }),
        }),
        car_states: vec![DesiredCarState {
            physics: Some(DesiredPhysics {
                location: Some(Vector3Partial::new(0.0, 0.0, 17.01)),
                rotation: Some(RotatorPartial::new(0.0, PI / 2.0, 0.0)),
                velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
                angular_velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
            }),
            boost_amount: Some(100.0),
            jumped: Some(false),
            double_jumped: Some(false),
        }],
    };
    stabilize_scenario(&rlbot, &initial_state);

    let f = File::create("collect.csv")?;
    let mut collector = Collector::new(f);

    let start = packets.next()?.GameInfo.TimeSeconds;

    let mut physics = rlbot.physicist();

    loop {
        let tick = physics.next_flat().unwrap();
        let packet = get_packet_and_inject_rigid_body_tick(&rlbot, tick)?;

        collector.write(tick)?;

        let time = packet.GameInfo.TimeSeconds - start;
        match scenarios::throttle(time, &packet) {
            Some(i) => rlbot.update_player_input(i, 0)?,
            None => break,
        }
    }

    // Stop the car before exiting (this is good safety advice IRL as well)
    rlbot.update_player_input(Default::default(), 0)?;

    Ok(())
}

fn start_match(rlbot: &rlbot::RLBot) -> Result<(), Box<Error>> {
    let mut match_settings = rlbot::ffi::MatchSettings {
        NumPlayers: 1,
        MutatorSettings: rlbot::ffi::MutatorSettings {
            MatchLength: rlbot::ffi::MatchLength::Unlimited,
            ..Default::default()
        },
        ..Default::default()
    };

    match_settings.PlayerConfiguration[0].Bot = true;
    match_settings.PlayerConfiguration[0].RLBotControlled = true;
    match_settings.PlayerConfiguration[0].set_name("Chell");

    rlbot.start_match(match_settings)?;
    Ok(())
}

fn stabilize_scenario(rlbot: &rlbot::RLBot, desired_game_state: &DesiredGameState) {
    let buffer = desired_game_state.serialize();
    rlbot.set_game_state(buffer.finished_data()).unwrap();
    sleep(Duration::from_millis(2000));
    rlbot.set_game_state(buffer.finished_data()).unwrap();
}
