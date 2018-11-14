#![cfg_attr(feature = "strict", deny(warnings))]

extern crate brain;
extern crate chrono;
extern crate collect;
extern crate log;
#[macro_use]
extern crate lazy_static;
extern crate common;
extern crate env_logger;
extern crate rlbot;

use brain::{Brain, EEG};
use chrono::Local;
use collect::{get_packet_and_inject_rigid_body_tick, Collector};
use common::ext::ExtendRLBot;
use std::{env, fs, panic, path::Path, thread::sleep, time::Duration};

mod logging;

fn main() {
    env_logger::Builder::new()
        .parse("brain") // Only log the brain crate
        .format(logging::format)
        .init();

    let rlbot = rlbot::init().unwrap();
    let rlbot: &rlbot::RLBot = Box::leak(Box::new(rlbot));

    let mut args = env::args();
    args.next().unwrap(); // Program name
    let use_framework = args.next().as_ref().map(String::as_str) == Some("--player-index");
    let player_index = if use_framework {
        args.next().unwrap().parse().unwrap()
    } else {
        start_match(&rlbot);
        0
    };

    if !use_framework {
        // In dev mode, halt on panics so they can't be ignored.
        run_bot(rlbot, player_index, use_framework);
    } else {
        // This is probably tournament mode, so we want to get back in action asap.
        deny_climate_change(|| run_bot(rlbot, player_index, use_framework));
    }
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

/// Keep running the given function until it doesn't panic.
fn deny_climate_change<R>(f: impl Fn() -> R) {
    loop {
        let result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
            f();
        }));
        match result {
            Ok(x) => return x,
            Err(_) => {
                println!("Panicked :( Let's try again?");
                sleep(Duration::from_millis(100));
            }
        }
    }
}

fn run_bot(rlbot: &'static rlbot::RLBot, player_index: i32, use_framework: bool) {
    let field_info = rlbot.get_field_info().unwrap();
    let brain = match Brain::infer_game_mode(&field_info) {
        rlbot::ffi::GameMode::Soccer => Brain::soccar(),
        rlbot::ffi::GameMode::Dropshot => Brain::dropshot(rlbot),
        rlbot::ffi::GameMode::Hoops => Brain::hoops(rlbot),
        mode => panic!("unexpected game mode {:?}", mode),
    };

    let collector = if !use_framework {
        Some(create_collector())
    } else {
        None
    };
    let eeg = EEG::new();
    let mut bot = FormulaNone::new(&field_info, collector, eeg, brain);
    bot.set_player_index(player_index);
    bot_loop(&rlbot, player_index, &mut bot);
}

fn bot_loop(rlbot: &rlbot::RLBot, player_index: i32, bot: &mut FormulaNone) {
    let mut physics = rlbot.physicist();

    loop {
        let rigid_body_tick = physics.next_flat().unwrap();
        let packet = get_packet_and_inject_rigid_body_tick(&rlbot, rigid_body_tick).unwrap();
        let input = bot.tick(rigid_body_tick, &packet);
        rlbot.update_player_input(input, player_index).unwrap();
    }
}

fn create_collector() -> Collector {
    let filename = Local::now()
        .format("logs/play-%Y-%m-%d_%H.%M.%S.csv")
        .to_string();
    let file = fs::File::create(&filename).unwrap();
    let collector = Collector::new(file);

    // Link a consistently-named file, for convenience.
    let link = Path::new("logs/play.csv");
    if link.exists() {
        fs::remove_file(link).unwrap();
    }
    fs::hard_link(filename, link).unwrap();

    collector
}

struct FormulaNone<'a> {
    field_info: &'a rlbot::ffi::FieldInfo,
    collector: Option<collect::Collector>,
    eeg: EEG,
    brain: Brain,
}

impl<'a> FormulaNone<'a> {
    fn new(
        field_info: &'a rlbot::ffi::FieldInfo,
        collector: Option<collect::Collector>,
        eeg: brain::EEG,
        brain: brain::Brain,
    ) -> Self {
        Self {
            field_info,
            collector,
            eeg,
            brain,
        }
    }

    fn set_player_index(&mut self, player_index: i32) {
        self.brain.set_player_index(player_index)
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

        let input = self.brain.tick(self.field_info, packet, &mut self.eeg);

        if let Some(collector) = &mut self.collector {
            collector.write(rigid_body_tick).unwrap();
        }
        self.eeg.show(&packet);

        input
    }
}
