use crate::{
    behavior::higher_order::{Fuse, NullBehavior},
    brain::Brain,
    eeg::{Event, EEG},
    strategy::{Behavior, Runner, Soccar, Team},
};
use brain_test_data::OneVOneScenario;
use collect::{
    get_packet_and_inject_rigid_body_tick, RecordingPlayerTick, RecordingRigidBodyState,
    RecordingTick,
};
use common::{ext::ExtendRLBot, prelude::*};
use lazy_static::lazy_static;
use nalgebra::{Point2, Point3, Rotation3, UnitQuaternion, Vector3};
use ordered_float::NotNan;
use std::{
    collections::HashSet,
    f32::consts::PI,
    fs::File,
    panic,
    path::Path,
    sync::{Arc, Barrier, Mutex, MutexGuard},
    thread::{self, sleep},
    time::Duration,
};

const RECORDING_DISTANCE_THRESHOLD: f32 = 25.0;
const STATE_SET_DEBOUNCE: i32 = 10;

pub struct TestRunner {
    behavior: Option<Box<FnMut(&rlbot::ffi::LiveDataPacket) -> Box<Behavior> + Send>>,
    ball_recording: Option<(Vec<f32>, Vec<RecordingRigidBodyState>)>,
    car_inital_state: Option<(RecordingRigidBodyState, f32)>,
    enemy_recording: Option<(Vec<f32>, Vec<RecordingPlayerTick>)>,
    enemy_initial_boost: f32,
}

impl TestRunner {
    const DEFAULT_STARTING_BOOST: f32 = 100.0;

    pub fn new() -> Self {
        Self {
            behavior: None,
            ball_recording: None,
            car_inital_state: None,
            enemy_recording: None,
            enemy_initial_boost: Self::DEFAULT_STARTING_BOOST,
        }
    }

    pub fn scenario(mut self, scenario: TestScenario) -> Self {
        self = self.ball(vec![0.0], vec![scenario.ball()]);
        self = self.car(scenario.car());
        self = self.starting_boost(scenario.boost as f32);
        self = self.enemy(vec![0.0], vec![Default::default()], vec![scenario.enemy()]);
        self = self.enemy_starting_boost(scenario.boost as f32);
        self
    }

    fn ball(
        mut self,
        times: impl Into<Vec<f32>>,
        states: impl Into<Vec<RecordingRigidBodyState>>,
    ) -> Self {
        self.ball_recording = Some((times.into(), states.into()));
        self
    }

    fn car(mut self, state: RecordingRigidBodyState) -> Self {
        self.car_inital_state = Some((state, Self::DEFAULT_STARTING_BOOST));
        self
    }

    pub fn starting_boost(mut self, boost: f32) -> Self {
        self.car_inital_state.as_mut().unwrap().1 = boost;
        self
    }

    pub fn enemy_starting_boost(mut self, boost: f32) -> Self {
        self.enemy_initial_boost = boost;
        self
    }

    fn enemy(
        mut self,
        times: impl Into<Vec<f32>>,
        inputs: impl Into<Vec<rlbot::ffi::PlayerInput>>,
        states: impl Into<Vec<RecordingRigidBodyState>>,
    ) -> Self {
        let ticks = inputs
            .into()
            .iter()
            .zip(states.into())
            .map(|(input, state)| RecordingPlayerTick {
                input: input.clone(),
                state: state.clone(),
            })
            .collect();
        self.enemy_recording = Some((times.into(), ticks));
        self
    }

    /// Replay the ball and enemy from a 1v1 recording. Lock the ball to the
    /// recording until the timestamp given by `ball_release`, and afterwards
    /// let it behave naturally.
    pub fn one_v_one(mut self, scenario: &OneVOneScenario, ball_release: f32) -> Self {
        let ball_release_index = scenario
            .times
            .iter()
            .position(|&t| t >= ball_release)
            .unwrap_or(scenario.times.len());
        self = self.ball(
            &scenario.times[..ball_release_index],
            &scenario.ball_states[..ball_release_index],
        );
        self = self.car(scenario.car_initial_state.clone());
        self = self.enemy(scenario.times, scenario.enemy_inputs, scenario.enemy_states);
        self
    }

    pub fn behavior(mut self, behavior: impl Behavior + Send + 'static) -> Self {
        // Use an option as a workaround for FnOnce being uncallable
        // https://github.com/rust-lang/rust/issues/28796
        let mut behavior = Some(Box::new(behavior));
        self.behavior = Some(Box::new(move |_| behavior.take().unwrap()));
        self
    }

    pub fn soccar(mut self) -> Self {
        self.behavior = Some(Box::new(|_| Box::new(Runner::new(Soccar::new()))));
        self
    }

    #[deprecated(note = "Do not commit references to ephemeral files.")]
    pub fn preview_recording(
        mut self,
        path: impl AsRef<Path>,
        start_time: f32,
        ball_duration: f32,
        total_duration: f32,
    ) -> Self {
        let ball_release = start_time + ball_duration;
        let stop_time = start_time + total_duration;

        let ticks: Vec<_> = RecordingTick::parse(File::open(path).unwrap())
            // Sometimes there are bogus rows at the start of the file from
            // before the game was restarted and the time was reset. Skip them.
            .skip_while(|r| r.time > stop_time)
            .skip_while(|r| r.time < start_time)
            .take_while(|r| r.time < stop_time)
            .collect();
        let times: Vec<_> = ticks.iter().map(|t| t.time).collect();
        let ball: Vec<_> = ticks.iter().map(|t| t.ball.clone()).collect();
        let enemy_ticks: Vec<_> = ticks.iter().map(|t| t.players[1].clone()).collect();

        let ball_release_index = times
            .iter()
            .position(|&t| t >= ball_release)
            .unwrap_or(times.len());

        self = self.ball(&times[..ball_release_index], &ball[..ball_release_index]);
        self.car_inital_state = Some((
            ticks[0].players[0].state.clone(),
            Self::DEFAULT_STARTING_BOOST,
        ));
        self.enemy_recording = Some((times, enemy_ticks));
        self
    }

    pub fn run(self) -> RunningTest {
        let ball = match self.ball_recording {
            Some((times, states)) => BallScenario::new(times, states),
            None => panic!(),
        };

        let car = match self.car_inital_state {
            Some(state) => CarScenario::single_tick(state.0, state.1),
            None => panic!(),
        };

        let enemy = match self.enemy_recording {
            Some((times, ticks)) => CarScenario::new(times, ticks, self.enemy_initial_boost),
            None => panic!(),
        };

        let mut behavior = self
            .behavior
            .unwrap_or_else(|| Box::new(|_| Box::new(NullBehavior::new())));

        let ready_wait = Arc::new(Barrier::new(2));
        let ready_wait_send = ready_wait.clone();
        let (messages_tx, messages_rx) = crossbeam_channel::unbounded();
        let thread = thread::spawn(move || {
            test_thread(
                ball,
                car,
                enemy,
                |p| behavior(p),
                ready_wait_send,
                messages_rx,
            )
        });
        ready_wait.wait();

        RunningTest {
            messages: messages_tx,
            join_handle: Some(thread),
        }
    }

    pub fn run_for_millis(self, millis: u64) -> RunningTest {
        let test = self.run();
        test.sleep_millis(millis);
        test
    }
}

pub struct RunningTest {
    messages: crossbeam_channel::Sender<Message>,
    join_handle: Option<thread::JoinHandle<()>>,
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
        sleep(Duration::from_millis(millis))
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
}

enum Message {
    SniffPacket(crossbeam_channel::Sender<rlbot::ffi::LiveDataPacket>),
    SetBehavior(Box<Behavior + Send>),
    HasScored(crossbeam_channel::Sender<bool>),
    EnemyHasScored(crossbeam_channel::Sender<bool>),
    ExamineEEG(Box<Fn(&EEG) + Send>),
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
    ball_scenario: BallScenario,
    car_scenario: CarScenario,
    enemy_scenario: CarScenario,
    behavior: impl FnOnce(&rlbot::ffi::LiveDataPacket) -> Box<Behavior>,
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

    let mut eeg = EEG::new().with_tracked_events();
    let mut brain = Brain::with_behavior(NullBehavior::new());
    brain.set_player_index(0);

    let mut packets = rlbot.packeteer();
    let mut physicist = rlbot.physicist();

    // Wait for RoundActive
    while !packets.next().unwrap().GameInfo.RoundActive {}

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

    let mut ball = BallDirector::new(ball_scenario, first_packet.GameInfo.TimeSeconds);
    let mut enemy = CarDirector::new(enemy_scenario, 1, first_packet.GameInfo.TimeSeconds);

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
    sleep(Duration::from_millis(1000));
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

fn rotator(r: UnitQuaternion<f32>) -> rlbot::state::RotatorPartial {
    let (pitch, yaw, roll) = r.to_rotation_matrix().to_unreal_angles();
    rlbot::state::RotatorPartial::new()
        .pitch(pitch)
        .yaw(yaw)
        .roll(roll)
}

pub struct TestScenario {
    pub ball_loc: Point3<f32>,
    pub ball_rot: Rotation3<f32>,
    pub ball_vel: Vector3<f32>,
    pub ball_ang_vel: Vector3<f32>,
    pub car_loc: Point3<f32>,
    pub car_rot: Rotation3<f32>,
    pub car_vel: Vector3<f32>,
    pub car_ang_vel: Vector3<f32>,
    pub enemy_loc: Point3<f32>,
    pub enemy_rot: Rotation3<f32>,
    pub enemy_vel: Vector3<f32>,
    pub enemy_ang_vel: Vector3<f32>,
    /// The amount of boost for both players, from 0 to 100.
    pub boost: u8,
}

impl Default for TestScenario {
    fn default() -> Self {
        TestScenario {
            ball_loc: Point3::new(0.0, 0.0, 92.74),
            ball_rot: Rotation3::from_unreal_angles(0.0, 0.0, 0.0),
            ball_vel: Vector3::new(0.0, 0.0, 0.0),
            ball_ang_vel: Vector3::new(0.0, 0.0, 0.0),
            car_loc: Point3::new(0.0, 0.0, 17.01),
            car_rot: Rotation3::from_unreal_angles(0.0, PI / 2.0, 0.0),
            car_vel: Vector3::new(0.0, 0.0, 0.0),
            car_ang_vel: Vector3::new(0.0, 0.0, 0.0),
            enemy_loc: Point3::new(6000.0, 6000.0, 0.0),
            enemy_rot: Rotation3::from_unreal_angles(0.0, 0.0, 0.0),
            enemy_vel: Vector3::new(0.0, 0.0, 0.0),
            enemy_ang_vel: Vector3::new(0.0, 0.0, 0.0),
            boost: 100,
        }
    }
}

impl TestScenario {
    /// This is a development-only convenience function that lets you load a
    /// scenario directly from a saved gameplay recording.
    #[deprecated(note = "Use TestScenario::new() instead when writing actual tests.")]
    pub fn from_recorded_row(filename: impl AsRef<Path>, time: f32) -> Self {
        let file = File::open(filename).unwrap();
        let tick = RecordingTick::parse(file)
            .find(|r| time <= r.time && r.time < time + 1.0)
            .unwrap();
        let result = Self {
            ball_loc: tick.ball.loc,
            ball_rot: tick.ball.rot.to_rotation_matrix(),
            ball_vel: tick.ball.vel,
            ball_ang_vel: tick.ball.ang_vel,
            car_loc: tick.players[0].state.loc,
            car_rot: tick.players[0].state.rot.to_rotation_matrix(),
            car_vel: tick.players[0].state.vel,
            car_ang_vel: tick.players[0].state.ang_vel,
            enemy_loc: tick.players[1].state.loc,
            enemy_rot: tick.players[1].state.rot.to_rotation_matrix(),
            enemy_vel: tick.players[1].state.vel,
            enemy_ang_vel: tick.players[1].state.ang_vel,
            ..Default::default()
        };
        println!("{}", result.to_source());
        return result;
    }

    fn ball(&self) -> RecordingRigidBodyState {
        RecordingRigidBodyState {
            loc: Point3::from(self.ball_loc),
            rot: UnitQuaternion::from_rotation_matrix(&self.ball_rot),
            vel: self.ball_vel,
            ang_vel: self.ball_ang_vel,
        }
    }

    fn car(&self) -> RecordingRigidBodyState {
        RecordingRigidBodyState {
            loc: Point3::from(self.car_loc),
            rot: UnitQuaternion::from_rotation_matrix(&self.car_rot),
            vel: self.car_vel,
            ang_vel: self.car_ang_vel,
        }
    }

    fn enemy(&self) -> RecordingRigidBodyState {
        RecordingRigidBodyState {
            loc: Point3::from(self.enemy_loc),
            rot: UnitQuaternion::from_rotation_matrix(&self.enemy_rot),
            vel: self.enemy_vel,
            ang_vel: self.enemy_ang_vel,
        }
    }

    fn to_source(&self) -> String {
        format!(
            "TestScenario {{
            ball_loc: Point3::new({}, {}, {}),
            ball_vel: Vector3::new({}, {}, {}),
            car_loc: Point3::new({}, {}, {}),
            car_rot: Rotation3::from_unreal_angles({}, {}, {}),
            car_vel: Vector3::new({}, {}, {}),
            ..Default::default()
        }}",
            self.ball_loc.x,
            self.ball_loc.y,
            self.ball_loc.z,
            self.ball_vel.x,
            self.ball_vel.y,
            self.ball_vel.z,
            self.car_loc.x,
            self.car_loc.y,
            self.car_loc.z,
            self.car_rot.pitch(),
            self.car_rot.yaw(),
            self.car_rot.roll(),
            self.car_vel.x,
            self.car_vel.y,
            self.car_vel.z,
        )
    }
}

struct BallScenario {
    times: Vec<NotNan<f32>>,
    states: Vec<RecordingRigidBodyState>,
}

impl BallScenario {
    fn new(times: Vec<impl Into<NotNan<f32>>>, states: Vec<RecordingRigidBodyState>) -> Self {
        Self {
            times: times.into_iter().map(Into::into).collect(),
            states,
        }
    }

    fn initial_state(&self) -> &RecordingRigidBodyState {
        &self.states[0]
    }
}

struct BallDirector {
    scenario: BallScenario,
    start_time: f32,
    frames_since_state_set: i32,
}

impl BallDirector {
    fn new(scenario: BallScenario, start_time: f32) -> Self {
        Self {
            scenario,
            start_time,
            frames_since_state_set: 0,
        }
    }

    fn tick(&mut self, rlbot: &rlbot::RLBot, packet: &rlbot::ffi::LiveDataPacket) {
        self.frames_since_state_set += 1;
        if self.frames_since_state_set < STATE_SET_DEBOUNCE {
            return;
        }

        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;
        let data_time = self.scenario.times[0] + elapsed;
        let index = match self.scenario.times.binary_search(&data_time) {
            Ok(i) => i,
            Err(0) => 0,
            Err(i) if i == self.scenario.times.len() => return,
            Err(i) => i - 1,
        };

        let state = &self.scenario.states[index];
        let current_loc = packet.GameBall.Physics.loc();
        if (state.loc - current_loc).norm() >= RECORDING_DISTANCE_THRESHOLD {
            let ball_state = rlbot::state::DesiredBallState::new().physics(
                rlbot::state::DesiredPhysics::new()
                    .location(state.loc)
                    .rotation(rotator(state.rot))
                    .velocity(state.vel)
                    .angular_velocity(state.ang_vel),
            );
            let game_state = rlbot::state::DesiredGameState::new().ball_state(ball_state);

            rlbot.set_game_state_struct(game_state).unwrap();
        }
    }
}

struct CarScenario {
    times: Vec<NotNan<f32>>,
    ticks: Vec<RecordingPlayerTick>,
    starting_boost: f32,
}

impl CarScenario {
    fn new(
        times: Vec<impl Into<NotNan<f32>>>,
        ticks: Vec<RecordingPlayerTick>,
        starting_boost: f32,
    ) -> Self {
        Self {
            times: times.into_iter().map(Into::into).collect(),
            ticks,
            starting_boost,
        }
    }

    fn single_tick(state: RecordingRigidBodyState, starting_boost: f32) -> Self {
        Self {
            times: vec![NotNan::new(0.0).unwrap()],
            ticks: vec![RecordingPlayerTick {
                input: Default::default(),
                state,
            }],
            starting_boost,
        }
    }

    fn initial_state(&self) -> &RecordingRigidBodyState {
        &self.ticks[0].state
    }
}

struct CarDirector {
    scenario: CarScenario,
    player_index: i32,
    start_time: f32,
    frames_since_state_set: i32,
}

impl CarDirector {
    fn new(scenario: CarScenario, player_index: i32, start_time: f32) -> Self {
        Self {
            scenario,
            player_index,
            start_time,
            frames_since_state_set: 0,
        }
    }

    fn tick(&mut self, rlbot: &rlbot::RLBot, packet: &rlbot::ffi::LiveDataPacket) {
        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;
        let data_time = self.scenario.times[0] + elapsed;
        let index = match self.scenario.times.binary_search(&data_time) {
            Ok(i) => i,
            Err(0) => 0,
            Err(i) if i == self.scenario.times.len() => return,
            Err(i) => i - 1,
        };
        let tick = &self.scenario.ticks[index];

        rlbot
            .update_player_input(tick.input, self.player_index)
            .unwrap();

        self.frames_since_state_set += 1;
        if self.frames_since_state_set < STATE_SET_DEBOUNCE {
            return;
        }

        let current_loc = packet.GameCars[self.player_index as usize].Physics.loc();
        if (tick.state.loc - current_loc).norm() >= RECORDING_DISTANCE_THRESHOLD {
            let car_state = rlbot::state::DesiredCarState::new().physics(
                rlbot::state::DesiredPhysics::new()
                    .location(tick.state.loc)
                    .rotation(rotator(tick.state.rot))
                    .velocity(tick.state.vel)
                    .angular_velocity(tick.state.ang_vel),
            );
            let game_state = rlbot::state::DesiredGameState::new()
                .car_state(self.player_index as usize, car_state);

            rlbot.set_game_state_struct(game_state).unwrap();
        }
    }
}
