extern crate brain;
extern crate chrono;
extern crate collect;
extern crate log;
#[macro_use]
extern crate lazy_static;
extern crate env_logger;
extern crate rlbot;

use brain::{Brain, EEG};
use chrono::Local;
use collect::Collector;
use std::fs::{hard_link, remove_file, File};
use std::path::Path;

mod logging;

fn main() {
    env_logger::Builder::new()
        .parse("brain") // Only log the brain crate
        .format(logging::format)
        .init();

    let rlbot = rlbot::init().unwrap();
    let match_settings = rlbot::MatchSettings {
        MutatorSettings: rlbot::MutatorSettings {
            MatchLength: rlbot::MatchLength::Unlimited,
            ..Default::default()
        },
        ..rlbot::MatchSettings::simple_1v1("Formula None", "All-Star")
    };
    rlbot.start_match(match_settings).unwrap();

    let mut packets = rlbot.packeteer();
    // Wait for RoundActive
    while !packets.next().unwrap().GameInfo.RoundActive {}

    let mut collector = create_collector();
    let mut eeg = EEG::new();
    let mut brain = Brain::with_root_behavior();

    loop {
        let packet = packets.next().unwrap();

        logging::STATE.lock().unwrap().game_time = Some(packet.GameInfo.TimeSeconds);

        let input = brain.tick(&packet, &mut eeg);
        rlbot.update_player_input(input, 0).unwrap();

        collector.write(&packet).unwrap();
        eeg.show(&packet);
    }
}

fn create_collector() -> Collector {
    let filename = Local::now()
        .format("logs/play-%Y-%m-%d_%H.%M.%S.csv")
        .to_string();
    let file = File::create(&filename).unwrap();
    let collector = Collector::new(file);

    // Link a consistently-named file, for convenience.
    let link = Path::new("logs/play.csv");
    if link.exists() {
        remove_file(link).unwrap();
    }
    hard_link(filename, link).unwrap();

    collector
}
