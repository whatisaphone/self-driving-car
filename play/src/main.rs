#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]

use brain::{Brain, EEG};
use chrono::Local;
use collect::{get_packet_and_inject_rigid_body_tick, Collector};
use common::ext::ExtendRLBot;
use std::{error::Error, fs, panic, path::PathBuf, thread::sleep, time::Duration};

mod logging;

fn main() {
    env_logger::Builder::new()
        .parse("brain") // Only log the brain crate
        .format(logging::format)
        .init();

    let StartArgs {
        init_options,
        should_start_match,
        should_recover_from_panics,
        should_log,
        player_index,
    } = parse_args().expect("Error parsing command-line arguments");

    let rlbot = rlbot::init_with_options(init_options).expect("Could not initialize RLBot");
    let rlbot: &rlbot::RLBot = Box::leak(Box::new(rlbot));

    if should_start_match {
        start_match(&rlbot).expect("Error starting match");
    }

    if !should_recover_from_panics {
        // In dev mode, halt on panics so they can't be ignored.
        run_bot(rlbot, player_index, should_log);
    } else {
        // This is probably tournament mode, so we want to get back in action asap.
        deny_climate_change(|| run_bot(rlbot, player_index, should_log));
    }
}

fn parse_args() -> Result<StartArgs, ()> {
    match rlbot::parse_framework_args()? {
        Some(args) => Ok(StartArgs {
            player_index: args.player_index,
            init_options: args.into(),
            should_start_match: false,
            should_recover_from_panics: true,
            should_log: false,
        }),
        None => Ok(StartArgs {
            init_options: rlbot::InitOptions::new(),
            should_start_match: true,
            should_recover_from_panics: false,
            should_log: true,
            player_index: 0,
        }),
    }
}

struct StartArgs {
    init_options: rlbot::InitOptions,
    should_start_match: bool,
    should_recover_from_panics: bool,
    should_log: bool,
    player_index: i32,
}

fn start_match(rlbot: &rlbot::RLBot) -> Result<(), Box<dyn Error>> {
    let match_settings = rlbot::ffi::MatchSettings {
        MutatorSettings: rlbot::ffi::MutatorSettings {
            MatchLength: rlbot::ffi::MatchLength::Unlimited,
            ..Default::default()
        },
        ..rlbot::ffi::MatchSettings::rlbot_vs_allstar("Formula None", "All-Star")
    };
    rlbot.start_match(match_settings)?;
    rlbot.wait_for_match_start()
}

/// Keep running the given function until it doesn't panic.
#[allow(clippy::redundant_closure)]
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

fn run_bot(rlbot: &'static rlbot::RLBot, player_index: i32, should_log: bool) {
    let field_info = rlbot.get_field_info().unwrap();
    let brain = match Brain::infer_game_mode(&field_info) {
        rlbot::ffi::GameMode::Soccer => Brain::soccar(),
        rlbot::ffi::GameMode::Dropshot => Brain::dropshot(rlbot),
        rlbot::ffi::GameMode::Hoops => Brain::hoops(rlbot),
        mode => panic!("unexpected game mode {:?}", mode),
    };

    let collector = if should_log {
        Some(create_collector())
    } else {
        None
    };
    let eeg = EEG::new();
    let mut bot = FormulaNone::new(&field_info, collector, eeg, brain);
    bot.set_player_index(player_index);
    bot_loop(&rlbot, player_index, &mut bot);
}

fn bot_loop(rlbot: &rlbot::RLBot, player_index: i32, bot: &mut FormulaNone<'_>) {
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
        rigid_body_tick: rlbot::flat::RigidBodyTick<'_>,
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
