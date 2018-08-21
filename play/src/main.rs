extern crate brain;
extern crate collect;
extern crate rlbot;

use brain::Brain;
use collect::Collector;
use std::fs::File;

fn main() {
    let rlbot = rlbot::init().unwrap();
    rlbot.start_match(rlbot::match_settings_1v1()).unwrap();

    let mut packets = rlbot.packeteer();

    // Wait for RoundActive
    while !packets.next().unwrap().GameInfo.RoundActive {}

    let f = File::create("play.csv").unwrap();
    let mut collector = Collector::new(f);

    let mut brain = Brain::new();

    loop {
        let packet = packets.next().unwrap();

        collector.write(&packet).unwrap();

        let input = brain.tick(&packet);
        rlbot.update_player_input(input, 0).unwrap();
    }
}
