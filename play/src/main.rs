extern crate brain;
extern crate chrono;
extern crate collect;
extern crate rlbot;

use brain::Brain;
use chrono::Local;
use collect::Collector;
use std::fs::{hard_link, remove_file, File};
use std::path::Path;

fn main() {
    let rlbot = rlbot::init().unwrap();
    rlbot.start_match(rlbot::match_settings_1v1()).unwrap();
    let mut packets = rlbot.packeteer();
    // Wait for RoundActive
    while !packets.next().unwrap().GameInfo.RoundActive {}

    let mut collector = create_collector();
    let mut brain = Brain::with_root_behavior();

    loop {
        let packet = packets.next().unwrap();

        let input = brain.tick(&packet);
        rlbot.update_player_input(input, 0).unwrap();

        collector.write(&packet).unwrap();
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
