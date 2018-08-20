extern crate brain;
extern crate rlbot;
use brain::Brain;

fn main() {
    rlbot::init().unwrap();
    rlbot::start_match(rlbot::match_settings_1v1()).unwrap();

    let mut packets = rlbot::LiveDataPackets::new();

    // Wait for RoundActive
    while !packets.wait().unwrap().GameInfo.RoundActive {}

    let mut brain = Brain::new();

    loop {
        let packet = packets.wait().unwrap();
        let input = brain.tick(&packet);
        rlbot::update_player_input(input, 0).unwrap();
    }
}
