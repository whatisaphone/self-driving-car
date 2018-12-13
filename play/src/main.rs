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
use std::{env, error::Error, fs, panic, path::PathBuf, thread::sleep, time::Duration};

mod logging;

fn main() {
    env_logger::Builder::new()
        .parse("brain") // Only log the brain crate
        .format(logging::format)
        .init();

    let rlbot = rlbot::init().expect("Could not initialize RLBot");
    let rlbot: &rlbot::RLBot = Box::leak(Box::new(rlbot));

    let (use_framework, should_start_match, player_index) =
        parse_args().expect("Error parsing command-line arguments");

    if should_start_match {
        start_match(&rlbot).expect("Error starting match");
    }

    if !use_framework {
        // In dev mode, halt on panics so they can't be ignored.
        run_bot(rlbot, player_index, use_framework);
    } else {
        // This is probably tournament mode, so we want to get back in action asap.
        deny_climate_change(|| run_bot(rlbot, player_index, use_framework));
    }
}

fn parse_args() -> Result<(bool, bool, i32), ()> {
    let mut args = env::args();
    args.next().ok_or(())?; // Program name
    let use_framework = args.next().as_ref().map(String::as_str) == Some("--player-index");
    let (should_start_match, player_index);
    if use_framework {
        should_start_match = false;
        player_index = args.next().ok_or(())?.parse().map_err(|_| ())?
    } else {
        should_start_match = true;
        player_index = 0;
    }
    Ok((use_framework, should_start_match, player_index))
}

fn start_match(rlbot: &rlbot::RLBot) -> Result<(), Box<Error>> {
    let match_settings = rlbot::ffi::MatchSettings {
        MutatorSettings: rlbot::ffi::MutatorSettings {
            MatchLength: rlbot::ffi::MatchLength::Unlimited,
            ..Default::default()
        },
        ..rlbot::ffi::MatchSettings::simple_1v1("Formula None", "All-Star")
    };
    rlbot.start_match(match_settings)?;
    rlbot.wait_for_match_start()
}

/// Keep running the given function until it doesn't panic.
fn deny_climate_change<R>(f: impl Fn() -> R) -> R {
    loop {
        let result = panic::catch_unwind(panic::AssertUnwindSafe(|| f()));
        match result {
            Ok(x) => break x,
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
    let directory = "logs";

    let now = Local::now().format("%Y-%m-%d_%H.%M.%S").to_string();
    let filename = format!("{}/play-{}.csv", directory, now);
    let file = fs::File::create(&filename).expect("Error creating game log file");

    // Link a consistently-named file, for convenience.
    let link = PathBuf::from(format!("{}/play.csv", directory));
    if link.exists() {
        fs::remove_file(&link).expect("Error deleting old game log hard link");
    }
    fs::hard_link(filename, &link).expect("Error creating game log hard link");

    Collector::new(file)
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
