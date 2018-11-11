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
    error::Error,
    fs::{hard_link, remove_file, File},
    mem,
    path::Path,
};

mod logging;

fn main() {
    env_logger::Builder::new()
        .parse("brain") // Only log the brain crate
        .format(logging::format)
        .init();

    let rlbot = rlbot::init().unwrap();
    let rlbot = Box::leak(Box::new(rlbot));

    let use_framework = env::args().len() != 1;
    if !use_framework {
        start_match(&rlbot);
    }

    let field_info = rlbot.get_field_info().unwrap();
    let brain = match infer_game_mode(&field_info) {
        rlbot::ffi::GameMode::Soccer => Brain::soccar(),
        rlbot::ffi::GameMode::Dropshot => Brain::dropshot(rlbot),
        mode => panic!("unexpected game mode {:?}", mode),
    };

    let collector = create_collector();
    let eeg = EEG::new();
    let mut bot = FormulaNone::new(collector, eeg, brain);
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
    rlbot.wait_for_match_start().unwrap();
}

trait RLBotExt {
    fn wait_for_match_start(&self) -> Result<(), Box<Error>>;
    fn get_field_info(&self) -> Result<rlbot::ffi::FieldInfo, Box<Error>>;
}

impl RLBotExt for rlbot::RLBot {
    /// Copy-pasted from unreleased rlbot 0.1.1.
    fn wait_for_match_start(&self) -> Result<(), Box<Error>> {
        let mut packets = self.packeteer();
        let mut count = 0;

        // Sometimes we get a few stray ticks from a previous game while the next game
        // is loading. Wait for RoundActive to stabilize before trusting it.
        while count < 5 {
            if packets.next()?.GameInfo.RoundActive {
                count += 1;
            } else {
                count = 0;
            }
        }
        Ok(())
    }

    fn get_field_info(&self) -> Result<rlbot::ffi::FieldInfo, Box<Error>> {
        let mut field_info = unsafe { mem::uninitialized() };
        self.update_field_info(&mut field_info)?;
        Ok(field_info)
    }
}

fn infer_game_mode(field_info: &rlbot::ffi::FieldInfo) -> rlbot::ffi::GameMode {
    match field_info.NumBoosts {
        0 => rlbot::ffi::GameMode::Dropshot,
        20 => rlbot::ffi::GameMode::Hoops,
        34 => rlbot::ffi::GameMode::Soccer,
        _ => panic!("unknown game mode"),
    }
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
