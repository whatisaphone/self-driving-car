use crate::{
    behavior::higher_order::{Fuse, NullBehavior},
    eeg::Event,
    integration_tests::{
        playback::{BallPlayback, BallRecording, CarPlayback, CarRecording},
        utils::rotator,
    },
    strategy::{Behavior, Team},
    Brain, EEG,
};
use collect::{get_packet_and_inject_rigid_body_tick, RecordingRigidBodyState};
use common::{prelude::*, ExtendRLBot};
use lazy_static::lazy_static;
use nalgebra::Point2;
use std::{
    collections::HashSet,
    panic,
    sync::{Arc, Barrier, Mutex, MutexGuard},
    thread,
    time::Duration,
};

pub struct RunningTest {
    pub messages: crossbeam_channel::Sender<Message>,
    pub join_handle: Option<thread::JoinHandle<()>>,
}

impl Drop for RunningTest {
    fn drop(&mut self) {
        self.messages.send(Message::Terminate);
        self.join_handle.take().unwrap().join().unwrap();
    }
}

impl RunningTest {
    // Right now this is just for convenience, but there's a possibility one day I
    // might tie it to packet.GameInfo.TimeSeconds so the tests still run properly
    // if sv_soccar_gamespeed is set to values other than 1 (if the stars align, of
    // course).
    pub fn sleep_millis(&self, millis: u64) {
        thread::sleep(Duration::from_millis(millis))
    }

    pub fn set_behavior(&self, behavior: impl Behavior + Send + 'static) {
        self.messages.send(Message::SetBehavior(Box::new(behavior)));
    }

    pub fn sniff_packet(&self) -> rlbot::ffi::LiveDataPacket {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.messages.send(Message::SniffPacket(tx));
        rx.recv().unwrap()
    }

    pub fn has_scored(&self) -> bool {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.messages.send(Message::HasScored(tx));
        rx.recv().unwrap()
    }

    pub fn enemy_has_scored(&self) -> bool {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.messages.send(Message::EnemyHasScored(tx));
        rx.recv().unwrap()
    }

    fn examine_eeg(&self, f: impl Fn(&EEG) + Send + 'static) {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.messages.send(Message::ExamineEEG(Box::new(move |eeg| {
            let unwind = panic::catch_unwind(panic::AssertUnwindSafe(|| f(eeg)));
            tx.send(unwind);
        })));
        match rx.recv().unwrap() {
            Ok(()) => {}
            Err(unwind) => panic::resume_unwind(unwind),
        };
    }

    pub fn examine_events(&self, f: impl Fn(&HashSet<Event>) + Send + 'static) {
        self.examine_eeg(move |eeg| f(eeg.events.as_ref().unwrap()));
    }

    pub fn spawn_thread(
        ball_scenario: BallRecording,
        car_scenario: CarRecording,
        enemy_scenario: CarRecording,
        behavior: impl FnOnce(&rlbot::ffi::LiveDataPacket) -> Box<dyn Behavior> + Send + 'static,
        ready_wait: Arc<Barrier>,
        messages: crossbeam_channel::Receiver<Message>,
    ) -> thread::JoinHandle<()> {
        thread::spawn(move || {
            test_thread(
                ball_scenario,
                car_scenario,
                enemy_scenario,
                behavior,
                ready_wait,
                messages,
            )
        })
    }
}

pub enum Message {
    SniffPacket(crossbeam_channel::Sender<rlbot::ffi::LiveDataPacket>),
    SetBehavior(Box<dyn Behavior + Send>),
    HasScored(crossbeam_channel::Sender<bool>),
    EnemyHasScored(crossbeam_channel::Sender<bool>),
    ExamineEEG(Box<dyn Fn(&EEG) + Send>),
    Terminate,
}

lazy_static! {
    static ref RLBOT_MUTEX: Mutex<Option<rlbot::RLBot>> = Mutex::new(None);
}

/// RLBot can only be initialized once, so keep a permanent instance around for
/// the tests (and leak it, don't worry, everything will be fine!)
fn unlock_rlbot_singleton() -> MutexGuard<'static, Option<rlbot::RLBot>> {
    let mut rlbot_guard = RLBOT_MUTEX.lock().unwrap();
    if rlbot_guard.is_none() {
        *rlbot_guard = Some(rlbot::init().unwrap());
    }
    rlbot_guard
}

fn test_thread(
    ball_scenario: BallRecording,
    car_scenario: CarRecording,
    enemy_scenario: CarRecording,
    behavior: impl FnOnce(&rlbot::ffi::LiveDataPacket) -> Box<dyn Behavior>,
    ready_wait: Arc<Barrier>,
    messages: crossbeam_channel::Receiver<Message>,
) {
    let rlbot_guard = unlock_rlbot_singleton();
    let rlbot = rlbot_guard.as_ref().unwrap();

    let mut match_settings = rlbot::ffi::MatchSettings {
        NumPlayers: 2,
        MutatorSettings: rlbot::ffi::MutatorSettings {
            MatchLength: rlbot::ffi::MatchLength::Unlimited,
            RespawnTimeOptions: rlbot::ffi::RespawnTimeOption::Disable_Goal_Reset,
            ..Default::default()
        },
        SkipReplays: true,
        ..Default::default()
    };

    match_settings.PlayerConfiguration[0].Bot = true;
    match_settings.PlayerConfiguration[0].RLBotControlled = true;
    match_settings.PlayerConfiguration[0].set_name("Subject");

    match_settings.PlayerConfiguration[1].Bot = true;
    match_settings.PlayerConfiguration[1].RLBotControlled = true;
    match_settings.PlayerConfiguration[1].set_name("Mushroom");
    match_settings.PlayerConfiguration[1].Team = 1;

    rlbot.start_match(match_settings).unwrap();

    let mut eeg = EEG::new();
    eeg.log_to_stdout();
    eeg.show_window();
    eeg.track_events();

    let mut brain = Brain::with_behavior(NullBehavior::new());
    brain.set_player_index(0);

    let mut packets = rlbot.packeteer();
    let mut physicist = rlbot.physicist();

    // Wait for things to stabilize.
    while !packets.next().unwrap().GameInfo.RoundActive {}
    while packets.next().unwrap().GameCars[0].Demolished {}

    for i in 0..match_settings.NumPlayers {
        rlbot.update_player_input(Default::default(), i).unwrap();
    }

    let field_info = rlbot.get_field_info().unwrap();

    setup_scenario(
        rlbot,
        ball_scenario.initial_state(),
        car_scenario.initial_state(),
        car_scenario.starting_boost,
        enemy_scenario.initial_state(),
        enemy_scenario.starting_boost,
    );

    let rigid_body_tick = physicist.next_flat().unwrap();
    let first_packet = get_packet_and_inject_rigid_body_tick(rlbot, rigid_body_tick).unwrap();

    brain.set_behavior(Fuse::new(behavior(&first_packet)), &mut eeg);
    ready_wait.wait();

    let mut ball = BallPlayback::new(ball_scenario, first_packet.GameInfo.TimeSeconds);
    let mut enemy = CarPlayback::new(enemy_scenario, 1, first_packet.GameInfo.TimeSeconds);

    'tick_loop: loop {
        let rigid_body_tick = physicist.next_flat().unwrap();
        let packet = get_packet_and_inject_rigid_body_tick(rlbot, rigid_body_tick).unwrap();

        ball.tick(rlbot, &packet);
        enemy.tick(rlbot, &packet);

        while let Some(message) = messages.try_recv() {
            match message {
                Message::SniffPacket(tx) => {
                    tx.send(packet);
                }
                Message::SetBehavior(behavior) => {
                    brain.set_behavior(Fuse::new(behavior), &mut eeg);
                }
                Message::HasScored(tx) => {
                    let first_score = first_packet.match_score();
                    let current_score = packet.match_score();
                    let team = Team::Blue.to_ffi() as usize;
                    tx.send(current_score[team] > first_score[team]);
                }
                Message::EnemyHasScored(tx) => {
                    let first_score = first_packet.match_score();
                    let current_score = packet.match_score();
                    let team = Team::Orange.to_ffi() as usize;
                    // This doesn't detect own goals, because of RLBot framework limitations. If
                    // there was a goal reset, conservatively assume that the enemy scored.
                    tx.send(current_score[team] > first_score[team] || is_kickoff(&packet));
                }
                Message::ExamineEEG(f) => {
                    f(&eeg);
                }
                Message::Terminate => {
                    break 'tick_loop;
                }
            }
        }

        eeg.begin(&packet);
        let input = brain.tick(&field_info, &packet, &mut eeg);
        rlbot.update_player_input(input, 0).unwrap();
        eeg.show(&packet);
        if let Some(chat) = eeg.quick_chat {
            if let Err(_) = rlbot.quick_chat(chat, 0) {
                log::warn!("could not quick chat {:?}", chat);
            }
        }
    }

    // For tidiness, make the cars stop moving when the test is finished.
    for i in 0..match_settings.NumPlayers {
        rlbot.update_player_input(Default::default(), i).unwrap();
    }
}

fn is_kickoff(packet: &rlbot::ffi::LiveDataPacket) -> bool {
    (packet.GameBall.Physics.loc_2d() - Point2::origin()).norm() < 1.0
}

fn setup_scenario(
    rlbot: &rlbot::RLBot,
    ball: &RecordingRigidBodyState,
    car: &RecordingRigidBodyState,
    car_boost: f32,
    enemy: &RecordingRigidBodyState,
    enemy_boost: f32,
) {
    let num_boosts = rlbot.get_field_info().unwrap().NumBoosts;

    set_state(rlbot, ball, car, car_boost, enemy, enemy_boost, num_boosts);
    // Wait for car suspension to settle to neutral, then set it again.
    thread::sleep(Duration::from_millis(1000));
    set_state(rlbot, ball, car, car_boost, enemy, enemy_boost, num_boosts);

    // Wait a few frames for the state to take effect.
    let mut packeteer = rlbot.packeteer();
    packeteer.next().unwrap();
    packeteer.next().unwrap();
}

fn set_state(
    rlbot: &rlbot::RLBot,
    ball: &RecordingRigidBodyState,
    car: &RecordingRigidBodyState,
    car_boost: f32,
    enemy: &RecordingRigidBodyState,
    enemy_boost: f32,
    num_boosts: i32,
) {
    let ball_state = rlbot::state::DesiredBallState::new().physics(
        rlbot::state::DesiredPhysics::new()
            .location(ball.loc)
            .rotation(rotator(ball.rot))
            .velocity(ball.vel)
            .angular_velocity(ball.ang_vel),
    );
    let car_state = rlbot::state::DesiredCarState::new()
        .physics(
            rlbot::state::DesiredPhysics::new()
                .location(car.loc)
                .rotation(rotator(car.rot))
                .velocity(car.vel)
                .angular_velocity(car.ang_vel),
        )
        .jumped(false)
        .double_jumped(false)
        .boost_amount(car_boost);
    let enemy_state = rlbot::state::DesiredCarState::new()
        .physics(
            rlbot::state::DesiredPhysics::new()
                .location(enemy.loc)
                .rotation(rotator(enemy.rot))
                .velocity(enemy.vel)
                .angular_velocity(enemy.ang_vel),
        )
        .boost_amount(enemy_boost);
    let mut game_state = rlbot::state::DesiredGameState::new()
        .ball_state(ball_state)
        .car_state(0, car_state)
        .car_state(1, enemy_state);

    for boost_index in 0..num_boosts as usize {
        game_state = game_state.boost_state(
            boost_index,
            rlbot::state::DesiredBoostState::new().respawn_time(0.0),
        );
    }

    rlbot.set_game_state_struct(game_state).unwrap();
}
