extern crate bakkesmod;
extern crate csv;
extern crate ratelimit;
extern crate rlbot;

use bakkesmod::BakkesMod;
use std::fs::File;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    run().unwrap();
}

fn run() -> Result<(), ()> {
    rlbot::init()?;
    rlbot::start_match(rlbot::match_settings_1v1())?;

    let mut packets = rlbot::LiveDataPackets::new();

    // Wait for RoundActive
    while !packets.wait()?.GameInfo.RoundActive {}

    // Zero out our input, just to be safe
    rlbot::update_player_input(Default::default(), 0)?;

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

    let f = File::create("data.csv").map_err(|_| ())?;
    let mut w = csv::Writer::from_writer(f);

    let start = packets.wait()?.GameInfo.TimeSeconds;

    loop {
        let packet = packets.wait()?;

        w.write_record(&[
            packet.GameInfo.TimeSeconds.to_string(),
            packet.GameCars[0].Physics.Location.X.to_string(),
            packet.GameCars[0].Physics.Location.Y.to_string(),
            packet.GameCars[0].Physics.Location.Z.to_string(),
            packet.GameCars[0].Physics.Rotation.Pitch.to_string(),
            packet.GameCars[0].Physics.Rotation.Yaw.to_string(),
            packet.GameCars[0].Physics.Rotation.Roll.to_string(),
            packet.GameCars[0].Physics.Velocity.X.to_string(),
            packet.GameCars[0].Physics.Velocity.Y.to_string(),
            packet.GameCars[0].Physics.Velocity.Z.to_string(),
            packet.GameCars[0].Physics.AngularVelocity.X.to_string(),
            packet.GameCars[0].Physics.AngularVelocity.Y.to_string(),
            packet.GameCars[0].Physics.AngularVelocity.Z.to_string(),
        ]).map_err(|_| ())?;

        // The action we want to record:
        if packet.GameInfo.TimeSeconds > start + 1.0 {
            let input = rlbot::PlayerInput {
                Throttle: 1.0,
                ..Default::default()
            };
            rlbot::update_player_input(input, 0)?;
        }

        // Once we have enough data, exit.
        if packet.GameInfo.TimeSeconds > start + 5.0 {
            break;
        }
    }

    Ok(())
}

fn stabilize_scenario(bakkesmod: &BakkesMod, message: &str) {
    for _ in 0..10 {
        bakkesmod.send(message);
        sleep(Duration::from_millis(100));
    }
    sleep(Duration::from_millis(1000));
}
