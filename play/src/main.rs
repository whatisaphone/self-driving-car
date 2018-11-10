#![cfg_attr(feature = "strict", deny(warnings))]

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
use collect::{get_packet_and_inject_rigid_body_tick, Collector};
use std::{
    env,
    fs::{hard_link, remove_file, File},
    path::Path,
};

mod logging;

fn main() {
    env_logger::Builder::new()
        .parse("brain") // Only log the brain crate
        .format(logging::format)
        .init();

    let collector = create_collector();
    let eeg = EEG::new();
    let brain = Brain::soccar();
    let mut bot = FormulaNone::new(collector, eeg, brain);

    let rlbot = rlbot::init().unwrap();

    let use_framework = env::args().len() != 1;
    if !use_framework {
        start_match(&rlbot);
    }
    bot_loop(&rlbot, &mut bot);
}

fn start_match(rlbot: &rlbot::RLBot) {
    let match_settings = rlbot::ffi::MatchSettings {
        MutatorSettings: rlbot::ffi::MutatorSettings {
            MatchLength: rlbot::ffi::MatchLength::Unlimited,
            ..Default::default()
        },
        ..rlbot::ffi::MatchSettings::simple_1v1("Formula None", "All-Star")
    };
    rlbot.start_match(match_settings).unwrap();

    let mut packets = rlbot.packeteer();
    while !packets.next().unwrap().GameInfo.RoundActive {}
}

fn bot_loop(rlbot: &rlbot::RLBot, bot: &mut FormulaNone) {
    bot.set_player_index(0);

    let mut physics = rlbot.physicist();

    loop {
        let rigid_body_tick = physics.next_flat().unwrap();
        let packet = get_packet_and_inject_rigid_body_tick(&rlbot, rigid_body_tick).unwrap();
        let input = bot.tick(rigid_body_tick, &packet);
        rlbot.update_player_input(input, 0).unwrap();
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

struct FormulaNone {
    collector: collect::Collector,
    eeg: EEG,
    brain: Brain,
}

impl FormulaNone {
    fn new(collector: collect::Collector, eeg: brain::EEG, brain: brain::Brain) -> Self {
        Self {
            collector,
            eeg,
            brain,
        }
    }

    fn set_player_index(&mut self, index: usize) {
        if index != 0 {
            unimplemented!();
        }
    }

    fn tick(
        &mut self,
        rigid_body_tick: rlbot::flat::RigidBodyTick,
        packet: &rlbot::ffi::LiveDataPacket,
    ) -> rlbot::ffi::PlayerInput {
        if !packet.GameInfo.RoundActive {
            return Default::default();
        }

        logging::STATE.lock().unwrap().game_time = Some(packet.GameInfo.TimeSeconds);

        self.eeg.begin(&packet);

        let input = self.brain.tick(packet, &mut self.eeg);

        self.collector.write(rigid_body_tick).unwrap();
        self.eeg.show(&packet);

        input
    }
}
