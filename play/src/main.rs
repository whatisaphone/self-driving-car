#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]

use crate::{banner::Banner, rlbot_ext::PacketeerExt};
use brain::{Brain, EEG};
use chrono::Local;
use collect::Collector;
use common::{
    ext::ExtendRLBot,
    halfway_house::{deserialize_game_tick_packet, translate_player_input},
};
use std::{error::Error, fs, panic, path::PathBuf, thread::sleep, time::Duration};

mod banner;
mod built;
mod logging;
mod rlbot_ext;

fn main() {
    println!("Self-Driving Car");
    println!("Built {}", built::BUILD_DATE);
    println!("See http://www.rlbot.org/ for more info!");

    env_logger::Builder::new()
        .parse_filters("brain,play") // Only log these crates
        .format(logging::format)
        .init();

    let StartArgs {
        init_options,
        should_start_match,
        should_recover_from_panics,
        log_game_data,
        log_to_stdout,
        show_window,
        player_index,
    } = parse_args().expect("Error parsing command-line arguments");

    let rlbot = rlbot::init_with_options(init_options).expect("Could not initialize RLBot");
    let rlbot: &rlbot::RLBot = Box::leak(Box::new(rlbot));

    if should_start_match {
        start_match(&rlbot).expect("Error starting match");
    }

    let run_the_bot = || {
        run_bot(
            rlbot,
            player_index,
            log_game_data,
            log_to_stdout,
            show_window,
        );
    };

    if !should_recover_from_panics {
        // In dev mode, halt on panics so they can't be ignored.
        run_the_bot();
    } else {
        // This is probably tournament mode, so we want to get back in action asap.
        deny_climate_change(run_the_bot);
    }
}

fn parse_args() -> Result<StartArgs, ()> {
    match rlbot::parse_framework_args()? {
        // If we're running in the framework:
        Some(args) => Ok(StartArgs {
            player_index: args.player_index,
            init_options: args.into(),
            should_start_match: false,
            should_recover_from_panics: true,
            log_game_data: false,
            log_to_stdout: false,
            show_window: false,
        }),
        // If we're running standalone:
        None => Ok(StartArgs {
            init_options: rlbot::InitOptions::new(),
            should_start_match: true,
            should_recover_from_panics: false,
            log_game_data: true,
            log_to_stdout: true,
            show_window: true,
            player_index: 0,
        }),
    }
}

struct StartArgs {
    init_options: rlbot::InitOptions,
    should_start_match: bool,
    should_recover_from_panics: bool,
    log_game_data: bool,
    log_to_stdout: bool,
    show_window: bool,
    player_index: i32,
}

fn start_match(rlbot: &rlbot::RLBot) -> Result<(), Box<dyn Error>> {
    let match_settings = rlbot::MatchSettings::rlbot_vs_allstar("Formula None", "All-Star")
        .mutator_settings(
            rlbot::MutatorSettings::new().match_length(rlbot::MatchLength::Unlimited),
        );
    rlbot.start_match(&match_settings)?;
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

fn run_bot(
    rlbot: &'static rlbot::RLBot,
    player_index: i32,
    log_game_data: bool,
    log_to_stdout: bool,
    show_window: bool,
) {
    let field_info = wait_for_field_info(rlbot);
    let brain = match Brain::infer_game_mode(field_info) {
        rlbot::GameMode::Soccer => Brain::soccar(),
        rlbot::GameMode::Dropshot => Brain::dropshot(rlbot),
        rlbot::GameMode::Hoops => Brain::hoops(rlbot),
        mode => panic!("unexpected game mode {:?}", mode),
    };

    let collector = if log_game_data {
        Some(create_collector())
    } else {
        None
    };
    let mut eeg = EEG::new();
    if log_to_stdout {
        eeg.log_to_stdout();
    }
    if show_window {
        eeg.show_window();
    }
    let mut bot = FormulaNone::new(rlbot, field_info, collector, eeg, brain);
    bot.set_player_index(player_index);
    bot_loop(&rlbot, player_index, &mut bot);
}

fn wait_for_field_info(rlbot: &rlbot::RLBot) -> rlbot::flat::FieldInfo<'_> {
    let mut packeteer = rlbot.packeteer();
    loop {
        packeteer.next().unwrap();
        if let Some(field_info) = rlbot.interface().update_field_info_flatbuffer() {
            if field_info.boostPads().is_some() {
                break field_info;
            }
        }
    }
}

fn bot_loop(rlbot: &rlbot::RLBot, player_index: i32, bot: &mut FormulaNone<'_>) {
    let mut packeteer = rlbot.packeteer();
    loop {
        let packet_flat = packeteer.next_flatbuffer_without_timeout().unwrap();
        let packet = deserialize_game_tick_packet(packet_flat);
        let rigid_body_tick = None; // No longer supported in the latest RLBot version.
        let (input, quick_chat) = bot.tick(rigid_body_tick, &packet);
        rlbot
            .update_player_input(player_index, &translate_player_input(&input))
            .unwrap();
        if let Some(chat) = quick_chat {
            if let Err(_) = rlbot.quick_chat(chat, player_index) {
                log::warn!("could not quick chat {:?}", chat);
            }
        }
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
    rlbot: &'static rlbot::RLBot,
    field_info: rlbot::flat::FieldInfo<'a>,
    collector: Option<collect::Collector>,
    eeg: EEG,
    brain: Brain,
    banner: Banner,
}

impl<'a> FormulaNone<'a> {
    fn new(
        rlbot: &'static rlbot::RLBot,
        field_info: rlbot::flat::FieldInfo<'a>,
        collector: Option<collect::Collector>,
        eeg: brain::EEG,
        brain: brain::Brain,
    ) -> Self {
        Self {
            rlbot,
            field_info,
            collector,
            eeg,
            brain,
            banner: Banner::new(),
        }
    }

    fn set_player_index(&mut self, player_index: i32) {
        self.brain.set_player_index(player_index)
    }

    fn tick(
        &mut self,
        rigid_body_tick: Option<rlbot::flat::RigidBodyTick<'_>>,
        packet: &common::halfway_house::LiveDataPacket,
    ) -> (
        common::halfway_house::PlayerInput,
        Option<rlbot::flat::QuickChatSelection>,
    ) {
        logging::STATE.lock().unwrap().game_time = Some(packet.GameInfo.TimeSeconds);
        self.banner.run(self.rlbot, packet);
        self.eeg.begin(&packet);

        let input = self.brain.tick(self.field_info, packet, &mut self.eeg);

        if let Some(collector) = &mut self.collector {
            if let Some(rigid_body_tick) = rigid_body_tick {
                collector.write(rigid_body_tick).unwrap();
            }
        }
        self.eeg.show(&packet);

        (input, self.eeg.quick_chat)
    }
}
