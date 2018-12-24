#![cfg_attr(feature = "strict", deny(warnings))]

use crate::{
    collector2::Collector,
    rlbot_ext::get_packet_and_inject_rigid_body_tick,
    scenarios::{Scenario, ScenarioStepResult},
};
use std::{error::Error, fs::File, thread::sleep, time::Duration};

mod collector2;
mod rlbot_ext;
mod scenarios;

pub fn main() -> Result<(), Box<Error>> {
    let rlbot = rlbot::init()?;

    // Zero out our input, just to be safe
    rlbot.update_player_input(Default::default(), 0)?;

    start_match(&rlbot)?;
    wait_for_match_start(&rlbot)?;

    run_scenario(&rlbot, scenarios::Coast::new())?;

    Ok(())
}

fn run_scenario(rlbot: &rlbot::RLBot, mut scenario: impl Scenario) -> Result<(), Box<Error>> {
    stabilize_scenario(&rlbot, scenario.initial_state());

    let f = File::create(format!("oven/data/{}.csv", scenario.name()))?;
    let mut collector = Collector::new(f);

    let mut packets = rlbot.packeteer();
    let start = packets.next()?.GameInfo.TimeSeconds;

    let mut physics = rlbot.physicist();

    loop {
        let tick = physics.next_flat().unwrap();
        let packet = get_packet_and_inject_rigid_body_tick(&rlbot, tick)?;

        let time = packet.GameInfo.TimeSeconds - start;
        match scenario.step(&rlbot, time, &packet)? {
            ScenarioStepResult::Ignore => {}
            ScenarioStepResult::Write => collector.write(tick)?,
            ScenarioStepResult::Finish => break,
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

fn wait_for_match_start(rlbot: &rlbot::RLBot) -> Result<(), Box<Error>> {
    let mut packets = rlbot.packeteer();
    let mut pings = 0;

    while pings < 5 {
        if packets.next()?.GameInfo.RoundActive {
            pings += 1;
        } else {
            pings = 0;
        }
    }
    Ok(())
}

fn stabilize_scenario(rlbot: &rlbot::RLBot, desired_game_state: rlbot::state::DesiredGameState) {
    rlbot
        .set_game_state_struct(desired_game_state.clone())
        .unwrap();
    sleep(Duration::from_millis(2000));
    rlbot.set_game_state_struct(desired_game_state).unwrap();
}
