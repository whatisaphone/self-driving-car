#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]

#[allow(deprecated)]
use crate::{
    collector::Collector,
    rlbot_ext::get_packet_and_inject_rigid_body_tick,
    scenarios::{Scenario, ScenarioStepResult},
};
use std::{error::Error, fs::File, thread::sleep, time::Duration};

mod collector;
mod rlbot_ext;
mod scenarios;

pub fn main() -> Result<(), Box<dyn Error>> {
    let rlbot = rlbot::init()?;

    // Zero out our input, just to be safe
    rlbot.update_player_input(0, &Default::default())?;

    start_match(&rlbot)?;

    run_scenario(&rlbot, scenarios::Jump::new())?;

    Ok(())
}

fn run_scenario(rlbot: &rlbot::RLBot, mut scenario: impl Scenario) -> Result<(), Box<dyn Error>> {
    stabilize_scenario(&rlbot, &scenario.initial_state());

    let f = File::create(format!("oven/data/{}.csv", scenario.name()))?;
    let mut collector = Collector::new(f);

    let mut packets = rlbot.packeteer();
    let start = packets.next()?.game_info.seconds_elapsed;

    let mut physics = rlbot.physicist();

    loop {
        let tick = physics.next_flat().unwrap();
        #[allow(deprecated)]
        let packet = get_packet_and_inject_rigid_body_tick(&rlbot, tick)?;

        let time = packet.GameInfo.TimeSeconds - start;
        match scenario.step(&rlbot, time, &packet)? {
            ScenarioStepResult::Ignore => {}
            ScenarioStepResult::Write => collector.write(tick)?,
            ScenarioStepResult::Finish => break,
        }
    }

    // Stop the car before exiting (this is good safety advice IRL as well)
    rlbot.update_player_input(0, &Default::default())?;

    Ok(())
}

fn start_match(rlbot: &rlbot::RLBot) -> Result<(), Box<dyn Error>> {
    rlbot.start_match(
        &rlbot::MatchSettings::new()
            .player_configurations(vec![rlbot::PlayerConfiguration::new(
                rlbot::PlayerClass::RLBotPlayer,
                "Chell",
                1,
            )])
            .mutator_settings(
                rlbot::MutatorSettings::new().match_length(rlbot::MatchLength::Unlimited),
            ),
    )?;
    rlbot.wait_for_match_start()
}

fn stabilize_scenario(rlbot: &rlbot::RLBot, desired_game_state: &rlbot::DesiredGameState) {
    rlbot.set_game_state(desired_game_state).unwrap();
    sleep(Duration::from_millis(2000));
    rlbot.set_game_state(desired_game_state).unwrap();
}
