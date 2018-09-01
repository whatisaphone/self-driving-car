use bakkesmod::BakkesMod;
use collector::Collector;
use rlbot;
use std::error::Error;
use std::fs::File;
use std::thread::sleep;
use std::time::Duration;

pub fn main() -> Result<(), Box<Error>> {
    let rlbot = rlbot::init()?;
    rlbot.start_match(rlbot::MatchSettings::simple_1v1())?;

    let mut packets = rlbot.packeteer();

    // Wait for RoundActive
    while !packets.next()?.GameInfo.RoundActive {}

    // Zero out our input, just to be safe
    rlbot.update_player_input(Default::default(), 0)?;

    let bakkesmod = BakkesMod::connect()?;
    let commands = [
        "ball location 2000 0 0",
        "ball velocity 0 0 0",
        "player 0 location 0 -5000 0",
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

        // The action we want to record:
        if packet.GameInfo.TimeSeconds < start + 1.0 {
            let input = Default::default();
            rlbot.update_player_input(input, 0)?;
        } else if packet.GameInfo.TimeSeconds < start + 3.0 {
            let input = rlbot::PlayerInput {
                Throttle: 1.0,
                Boost: true,
                ..Default::default()
            };
            rlbot.update_player_input(input, 0)?;
        } else if packet.GameInfo.TimeSeconds < start + 8.0 {
            let input = Default::default();
            rlbot.update_player_input(input, 0)?;
        } else {
            break;
        }
    }

    // Stop the car before exiting (just like IRL!)
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
