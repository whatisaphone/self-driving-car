#![cfg_attr(feature = "strict", deny(warnings))]

extern crate bakkesmod;
extern crate csv;
extern crate rlbot;

use bakkesmod::BakkesMod;
use collector::Collector;
use std::{error::Error, fs::File, thread::sleep, time::Duration};

mod collector;
mod scenarios;

pub fn main() -> Result<(), Box<Error>> {
    let rlbot = rlbot::init()?;
    rlbot.start_match(rlbot::ffi::MatchSettings::simple_1v1(
        "Collector",
        "Spectator",
    ))?;

    let mut packets = rlbot.packeteer();

    // Wait for RoundActive
    while !packets.next()?.GameInfo.RoundActive {}

    // Zero out our input, just to be safe
    rlbot.update_player_input(Default::default(), 0)?;

    let bakkesmod = BakkesMod::connect()?;
    let commands = [
        "ball location 2000 0 0",
        "ball velocity 0 0 0",
        "player 0 location 0 0 0",
        "player 0 rotation 0 16384 0",
        "player 0 velocity 0 0 0",
        "player 1 location 6000 6000 0",
        "player 1 rotation 0 0 0",
        "player 1 velocity 0 0 0",
        "boost set 100",
    ];
    stabilize_scenario(&bakkesmod, &commands.join(";"));

    let f = File::create("collect.csv")?;
    let mut collector = Collector::new(f);

    let start = packets.next()?.GameInfo.TimeSeconds;

    loop {
        let packet = packets.next()?;

        collector.write(&packet)?;

        if !scenarios::aerial_60deg(&rlbot, packet.GameInfo.TimeSeconds - start, &packet)? {
            break;
        }
    }

    // Stop the car before exiting (this is good safety advice IRL as well)
    rlbot.update_player_input(Default::default(), 0)?;

    Ok(())
}

fn stabilize_scenario(bakkesmod: &BakkesMod, message: &str) {
    for _ in 0..10 {
        bakkesmod.send(message);
        sleep(Duration::from_millis(100));
    }
    sleep(Duration::from_millis(1000));
}
