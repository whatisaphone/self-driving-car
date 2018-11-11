use behavior::{Behavior, Fuse, NullBehavior};
use brain::Brain;
use brain_test_data::OneVOneScenario;
use collect::{
    get_packet_and_inject_rigid_body_tick, RecordingPlayerTick, RecordingRigidBodyState,
    RecordingTick,
};
use common::{ext::ExtendRLBot, prelude::*};
use crossbeam_channel;
use eeg::EEG;
use flatbuffers::{FlatBufferBuilder, WIPOffset};
use nalgebra::{Point3, Rotation3, UnitQuaternion, Vector3};
use ordered_float::NotNan;
use rlbot;
use std::{
    f32::consts::PI,
    fs::File,
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
}

/// Static API
impl TestRunner {
    pub fn start(behavior: impl Behavior + Send + 'static, scenario: TestScenario) -> RunningTest {
        Self::start2(scenario, |_| behavior)
    }

    pub fn start0(scenario: TestScenario) -> RunningTest {
        Self::start2(scenario, |_| NullBehavior::new())
    }

    pub fn start2<B, BF>(scenario: TestScenario, behavior: BF) -> RunningTest
    where
        B: Behavior + 'static,
        BF: FnOnce(&rlbot::ffi::LiveDataPacket) -> B + Send + 'static,
    {
        Self::new().scenario(scenario).behavior_fn(behavior).run()
    }
}

/// Builder API
impl TestRunner {
    pub fn new() -> Self {
        Self {
            behavior: None,
            ball_recording: None,
            car_inital_state: None,
            enemy_recording: None,
        }
    }

    pub fn scenario(mut self, scenario: TestScenario) -> Self {
        self = self.ball(vec![0.0], vec![scenario.ball()]);
        self = self.car(scenario.car());
        self = self.enemy(vec![0.0], vec![Default::default()], vec![scenario.enemy()]);
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
        self.car_inital_state = Some((state, 1.0));
        self
    }

    pub fn starting_boost(mut self, boost: f32) -> Self {
        self.car_inital_state.as_mut().unwrap().1 = boost;
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

    pub fn behavior_fn<B, BF>(mut self, behavior: BF) -> Self
    where
        B: Behavior + 'static,
        BF: FnOnce(&rlbot::ffi::LiveDataPacket) -> B + Send + 'static,
    {
        // Use an option as a workaround for FnOnce being uncallable
        // https://github.com/rust-lang/rust/issues/28796
        let mut behavior = Some(behavior);
        self.behavior = Some(Box::new(move |p| Box::new(behavior.take().unwrap()(p))));
        self
    }

    #[deprecated(note = "Do not commit references to ephemeral files.")]
    pub fn preview_recording(
        mut self,
        path: impl AsRef<Path>,
        start_time: f32,
        ball_release: f32,
        stop_time: f32,
    ) -> Self {
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
        self.car_inital_state = Some((ticks[0].players[0].state.clone(), 1.0));
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
            Some((times, ticks)) => CarScenario::new(times, ticks, 1.0),
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

    pub fn examine_eeg(&self, f: impl Fn(&EEG) + Send + 'static) {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.messages.send(Message::ExamineEEG(Box::new(move |eeg| {
            f(eeg);
            tx.send(());
        })));
        rx.recv().unwrap();
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

    let mut eeg = EEG::new();
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
    );

    let first_packet = {
        let rigid_body_tick = physicist.next_flat().unwrap();
        get_packet_and_inject_rigid_body_tick(rlbot, rigid_body_tick).unwrap()
    };

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
                    tx.send(current_score[0] > first_score[0]); // Index 0 means own team
                }
                Message::EnemyHasScored(tx) => {
                    let first_score = first_packet.match_score();
                    let current_score = packet.match_score();
                    tx.send(current_score[1] > first_score[1]); // Index 1 means enemy team
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

fn setup_scenario(
    rlbot: &rlbot::RLBot,
    ball: &RecordingRigidBodyState,
    car: &RecordingRigidBodyState,
    car_boost_amount: f32,
    enemy: &RecordingRigidBodyState,
) {
    set_state(rlbot, ball, car, car_boost_amount, enemy);
    // Wait for car suspension to settle to neutral, then set it again.
    sleep(Duration::from_millis(1000));
    set_state(rlbot, ball, car, car_boost_amount, enemy);

    // Wait a few frames for the state to take effect.
    rlbot.packeteer().next().unwrap();
    rlbot.packeteer().next().unwrap();
    rlbot.packeteer().next().unwrap();
    rlbot.packeteer().next().unwrap();
}

fn set_state(
    rlbot: &rlbot::RLBot,
    ball: &RecordingRigidBodyState,
    car: &RecordingRigidBodyState,
    car_boost_amount: f32,
    enemy: &RecordingRigidBodyState,
) {
    let mut builder = FlatBufferBuilder::new_with_capacity(1024);

    let physics = desired_physics(&mut builder, ball.loc, ball.rot, ball.vel, ball.ang_vel);
    let ball_state = {
        let mut b = rlbot::flat::DesiredBallStateBuilder::new(&mut builder);
        b.add_physics(physics);
        b.finish()
    };

    let physics = desired_physics(&mut builder, car.loc, car.rot, car.vel, car.ang_vel);
    let car_state = {
        let boost_amount = rlbot::flat::Float::new(car_boost_amount);
        let mut b = rlbot::flat::DesiredCarStateBuilder::new(&mut builder);
        b.add_physics(physics);
        b.add_boostAmount(&boost_amount);
        b.finish()
    };

    let physics = desired_physics(&mut builder, enemy.loc, enemy.rot, enemy.vel, enemy.ang_vel);
    let enemy_state = {
        let boost_amount = rlbot::flat::Float::new(100.0);
        let mut b = rlbot::flat::DesiredCarStateBuilder::new(&mut builder);
        b.add_physics(physics);
        b.add_boostAmount(&boost_amount);
        b.finish()
    };

    let car_states = builder.create_vector(&[car_state, enemy_state]);
    let desired_game_state = {
        let mut b = rlbot::flat::DesiredGameStateBuilder::new(&mut builder);
        b.add_ballState(ball_state);
        b.add_carStates(car_states);
        b.finish()
    };

    builder.finish(desired_game_state, None);
    rlbot.set_game_state(builder.finished_data()).unwrap()
}

fn desired_physics<'a, 'b>(
    builder: &'b mut FlatBufferBuilder<'a>,
    loc: Point3<f32>,
    rot: UnitQuaternion<f32>,
    vel: Vector3<f32>,
    ang_vel: Vector3<f32>,
) -> WIPOffset<rlbot::flat::DesiredPhysics<'a>> {
    let loc = vector3(builder, loc.coords);
    let rot = rotator(builder, rot);
    let vel = vector3(builder, vel);
    let ang_vel = vector3(builder, ang_vel);
    let mut b = rlbot::flat::DesiredPhysicsBuilder::new(builder);
    b.add_location(loc);
    b.add_rotation(rot);
    b.add_velocity(vel);
    b.add_angularVelocity(ang_vel);
    b.finish()
}

fn vector3<'a, 'b>(
    builder: &'b mut FlatBufferBuilder<'a>,
    v: Vector3<f32>,
) -> WIPOffset<rlbot::flat::Vector3Partial<'a>> {
    rlbot::flat::Vector3Partial::create(
        builder,
        &rlbot::flat::Vector3PartialArgs {
            x: Some(&rlbot::flat::Float::new(v.x)),
            y: Some(&rlbot::flat::Float::new(v.y)),
            z: Some(&rlbot::flat::Float::new(v.z)),
        },
    )
}

fn rotator<'a, 'b>(
    builder: &'b mut FlatBufferBuilder<'a>,
    r: UnitQuaternion<f32>,
) -> WIPOffset<rlbot::flat::RotatorPartial<'a>> {
    let rotation = r.rocket_league_munge().to_rotation_matrix();
    rlbot::flat::RotatorPartial::create(
        builder,
        &rlbot::flat::RotatorPartialArgs {
            pitch: Some(&rlbot::flat::Float::new(rotation.pitch())),
            yaw: Some(&rlbot::flat::Float::new(rotation.yaw())),
            roll: Some(&rlbot::flat::Float::new(rotation.roll())),
        },
    )
}

pub struct TestScenario {
    pub ball_loc: Vector3<f32>,
    pub ball_rot: Rotation3<f32>,
    pub ball_vel: Vector3<f32>,
    pub ball_ang_vel: Vector3<f32>,
    pub car_loc: Vector3<f32>,
    pub car_rot: Rotation3<f32>,
    pub car_vel: Vector3<f32>,
    pub car_ang_vel: Vector3<f32>,
    pub enemy_loc: Vector3<f32>,
    pub enemy_rot: Rotation3<f32>,
    pub enemy_vel: Vector3<f32>,
    pub enemy_ang_vel: Vector3<f32>,
    pub boost: u8,
}

impl Default for TestScenario {
    fn default() -> Self {
        TestScenario {
            ball_loc: Vector3::new(0.0, 0.0, 92.74),
            ball_rot: Rotation3::from_unreal_angles(0.0, 0.0, 0.0),
            ball_vel: Vector3::new(0.0, 0.0, 0.0),
            ball_ang_vel: Vector3::new(0.0, 0.0, 0.0),
            car_loc: Vector3::new(0.0, 0.0, 17.01),
            car_rot: Rotation3::from_unreal_angles(0.0, PI / 2.0, 0.0),
            car_vel: Vector3::new(0.0, 0.0, 0.0),
            car_ang_vel: Vector3::new(0.0, 0.0, 0.0),
            enemy_loc: Vector3::new(6000.0, 6000.0, 0.0),
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
    #[allow(dead_code)]
    #[deprecated(note = "Use TestScenario::new() instead when writing actual tests.")]
    pub fn from_recorded_row(filename: impl AsRef<Path>, time: f32) -> Self {
        let file = File::open(filename).unwrap();
        let tick = RecordingTick::parse(file)
            .find(|r| time <= r.time && r.time < time + 1.0)
            .unwrap();
        let result = Self {
            ball_loc: tick.ball.loc.coords,
            ball_rot: tick.ball.rot.to_rotation_matrix(),
            ball_vel: tick.ball.vel,
            ball_ang_vel: tick.ball.ang_vel,
            car_loc: tick.players[0].state.loc.coords,
            car_rot: tick.players[0].state.rot.to_rotation_matrix(),
            car_vel: tick.players[0].state.vel,
            car_ang_vel: tick.players[0].state.ang_vel,
            enemy_loc: tick.players[1].state.loc.coords,
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
            ball_loc: Vector3::new({}, {}, {}),
            ball_vel: Vector3::new({}, {}, {}),
            car_loc: Vector3::new({}, {}, {}),
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
        let current_loc = packet.GameBall.Physics.locp();
        if (state.loc - current_loc).norm() >= RECORDING_DISTANCE_THRESHOLD {
            let mut builder = FlatBufferBuilder::new_with_capacity(1024);

            let physics =
                desired_physics(&mut builder, state.loc, state.rot, state.vel, state.ang_vel);
            let desired_ball_state = {
                let mut b = rlbot::flat::DesiredBallStateBuilder::new(&mut builder);
                b.add_physics(physics);
                b.finish()
            };
            let desired_game_state = {
                let mut b = rlbot::flat::DesiredGameStateBuilder::new(&mut builder);
                b.add_ballState(desired_ball_state);
                b.finish()
            };

            builder.finish(desired_game_state, None);
            rlbot.set_game_state(builder.finished_data()).unwrap();
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

        let current_loc = packet.GameCars[self.player_index as usize].Physics.locp();
        if (tick.state.loc - current_loc).norm() >= RECORDING_DISTANCE_THRESHOLD {
            let mut builder = FlatBufferBuilder::new_with_capacity(1024);

            let mut car_states = Vec::new();

            // Push empty states to pad out the array so we target the right index
            for _ in 0..self.player_index {
                car_states.push(rlbot::flat::DesiredCarStateBuilder::new(&mut builder).finish());
            }

            let physics = desired_physics(
                &mut builder,
                tick.state.loc,
                tick.state.rot,
                tick.state.vel,
                tick.state.ang_vel,
            );
            {
                let boost_amount = rlbot::flat::Float::new(100.0);
                let mut b = rlbot::flat::DesiredCarStateBuilder::new(&mut builder);
                b.add_physics(physics);
                b.add_boostAmount(&boost_amount);
                car_states.push(b.finish());
            };
            let car_states = builder.create_vector(&car_states);
            let desired_game_state = {
                let mut b = rlbot::flat::DesiredGameStateBuilder::new(&mut builder);
                b.add_carStates(car_states);
                b.finish()
            };

            builder.finish(desired_game_state, None);
            rlbot.set_game_state(builder.finished_data()).unwrap();
        }
    }
}
