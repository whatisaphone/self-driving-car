use bakkesmod::BakkesMod;
use behavior::{Behavior, Fuse, NullBehavior};
use brain::Brain;
use collect::{ExtendRotation3, Snapshot};
use crossbeam_channel;
use csv;
use eeg::EEG;
use nalgebra::{Rotation3, Vector3};
use rlbot;
use std::{
    error::Error,
    f32::consts::PI,
    fs::File,
    path::Path,
    sync::{Arc, Barrier, Mutex, MutexGuard},
    thread::{self, sleep},
    time::Duration,
};

pub struct TestRunner {
    sniff_packet: crossbeam_channel::Sender<crossbeam_channel::Sender<rlbot::ffi::LiveDataPacket>>,
    set_behavior: crossbeam_channel::Sender<Box<Behavior + Send>>,
    has_scored: crossbeam_channel::Sender<crossbeam_channel::Sender<bool>>,
    messages: crossbeam_channel::Sender<Message>,
    terminate: Option<crossbeam_channel::Sender<()>>,
    join_handle: Option<thread::JoinHandle<()>>,
}

impl TestRunner {
    pub fn start(
        behavior: impl Behavior + Send + 'static,
        scenario: impl BakkesModCommand + Send + 'static,
    ) -> TestRunner {
        TestRunner::start2(scenario, |_| behavior)
    }

    pub fn start0(scenario: impl BakkesModCommand + Send + 'static) -> TestRunner {
        Self::start2(scenario, |_| NullBehavior::new())
    }

    pub fn start2<B, BF>(
        scenario: impl BakkesModCommand + Send + 'static,
        behavior: BF,
    ) -> TestRunner
    where
        B: Behavior + 'static,
        BF: FnOnce(&rlbot::ffi::LiveDataPacket) -> B + Send + 'static,
    {
        let ready_wait = Arc::new(Barrier::new(2));
        let ready_wait_send = ready_wait.clone();
        let (terminate_tx, terminate_rx) = crossbeam_channel::unbounded();
        let (sniff_packet_tx, sniff_packet_rx) = crossbeam_channel::unbounded();
        let (set_behavior_tx, set_behavior_rx) = crossbeam_channel::unbounded();
        let (has_scored_tx, has_scored_rx) = crossbeam_channel::unbounded();
        let (messages_tx, messages_rx) = crossbeam_channel::unbounded();
        let thread = thread::spawn(|| {
            test_thread(
                scenario,
                |p| Box::new(behavior(p)),
                ready_wait_send,
                sniff_packet_rx,
                set_behavior_rx,
                has_scored_rx,
                messages_rx,
                terminate_rx,
            )
        });

        ready_wait.wait();
        TestRunner {
            sniff_packet: sniff_packet_tx,
            set_behavior: set_behavior_tx,
            has_scored: has_scored_tx,
            messages: messages_tx,
            terminate: Some(terminate_tx),
            join_handle: Some(thread),
        }
    }
}

impl Drop for TestRunner {
    fn drop(&mut self) {
        self.terminate.take().unwrap().send(());
        self.join_handle.take().unwrap().join().unwrap();
    }
}

impl TestRunner {
    // Right now this is just for convenience, but there's a possibility one day I
    // might tie it to packet.GameInfo.TimeSeconds so the tests still run properly
    // if sv_soccar_gamespeed is set to values other than 1 (if the stars align, of
    // course).
    pub fn sleep_millis(&self, millis: u64) {
        sleep(Duration::from_millis(millis))
    }

    pub fn set_behavior(&self, behavior: impl Behavior + Send + 'static) {
        self.set_behavior.send(Box::new(behavior));
    }

    pub fn sniff_packet(&self) -> rlbot::ffi::LiveDataPacket {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sniff_packet.send(tx);
        rx.recv().unwrap()
    }

    pub fn has_scored(&self) -> bool {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.has_scored.send(tx);
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
    ExamineEEG(Box<Fn(&EEG) + Send>),
    EnemyHasScored(crossbeam_channel::Sender<bool>),
}

lazy_static! {
    static ref RLBOT_MUTEX: Mutex<Option<rlbot::RLBot>> = Mutex::new(None);
}

/// RLBot_Core_Interface.dll sometimes crashes if it's unloaded and then
/// reloaded, so just keep a permanent instance around for the tests (and
/// leak it, don't worry, everything will be fine!)
fn unlock_rlbot_singleton() -> MutexGuard<'static, Option<rlbot::RLBot>> {
    let mut rlbot_guard = RLBOT_MUTEX.lock().unwrap();
    if rlbot_guard.is_none() {
        *rlbot_guard = Some(rlbot::init().unwrap());
    }
    rlbot_guard
}

fn test_thread(
    scenario: impl BakkesModCommand,
    behavior: impl FnOnce(&rlbot::ffi::LiveDataPacket) -> Box<Behavior>,
    ready_wait: Arc<Barrier>,
    sniff_packet: crossbeam_channel::Receiver<
        crossbeam_channel::Sender<rlbot::ffi::LiveDataPacket>,
    >,
    set_behavior: crossbeam_channel::Receiver<Box<Behavior + Send>>,
    has_scored: crossbeam_channel::Receiver<crossbeam_channel::Sender<bool>>,
    messages: crossbeam_channel::Receiver<Message>,
    terminate: crossbeam_channel::Receiver<()>,
) {
    let rlbot_guard = unlock_rlbot_singleton();
    let rlbot = rlbot_guard.as_ref().unwrap();

    let mut match_settings = rlbot::ffi::MatchSettings {
        NumPlayers: 2,
        MutatorSettings: rlbot::ffi::MutatorSettings {
            MatchLength: rlbot::ffi::MatchLength::Unlimited,
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
    let mut brain = Brain::with_behavior(Box::new(NullBehavior::new()));

    let mut packets = rlbot.packeteer();
    // Wait for RoundActive
    while !packets.next().unwrap().GameInfo.RoundActive {}

    rlbot.update_player_input(Default::default(), 0).unwrap();

    setup_scenario(scenario);
    wait_for_bakkesmod_flush(&mut packets).unwrap();

    let first_packet = packets.next().unwrap();
    brain.set_behavior(Box::new(Fuse::new(behavior(&first_packet))), &mut eeg);
    ready_wait.wait();

    loop {
        let packet = packets.next().unwrap();
        eeg.begin(&packet);
        let input = brain.tick(&packet, &mut eeg);
        rlbot.update_player_input(input, 0).unwrap();
        eeg.show(&packet);

        if let Some(chan) = has_scored.try_recv() {
            let first_score = first_packet.match_score();
            let current_score = packet.match_score();
            chan.send(current_score[0] > first_score[0]); // Index 0 means blue team
        }

        if let Some(chan) = sniff_packet.try_recv() {
            chan.send(packet);
        }

        if let Some(behavior) = set_behavior.try_recv() {
            brain.set_behavior(Box::new(Fuse::new(behavior)), &mut eeg);
        }

        while let Some(message) = messages.try_recv() {
            match message {
                Message::ExamineEEG(f) => f(&eeg),
                Message::EnemyHasScored(tx) => {
                    let first_score = first_packet.match_score();
                    let current_score = packet.match_score();
                    tx.send(current_score[1] > first_score[1]); // Index 1 means enemy team
                }
            }
        }

        if let Some(()) = terminate.try_recv() {
            break;
        }
    }

    // For tidiness, make the car stop moving when the test is finished.
    rlbot.update_player_input(Default::default(), 0).unwrap();
}

fn setup_scenario(scenario: impl BakkesModCommand) {
    let bakkesmod = BakkesMod::connect().unwrap();

    let neutral = TestScenario {
        car_loc: Vector3::new(1000.0, 0.0, 17.01),
        enemy_loc: Vector3::new(-1000.0, 0.0, 17.01),
        boost: 0,
        ..Default::default()
    }
    .to_bakkesmod_command();

    for _ in 0..6 {
        bakkesmod.send(neutral.clone());
        sleep(Duration::from_millis(250));
    }

    let command = scenario.to_bakkesmod_command();
    bakkesmod.send(command);
}

/// Wait for the scenario to be returned from the game so we don't "leak" the
/// reset scenario to the tested code.
fn wait_for_bakkesmod_flush(packets: &mut rlbot::Packeteer) -> Result<(), Box<Error>> {
    for _ in 0..10 {
        let packet = packets.next()?;
        if (packet.GameCars[0].Physics.Location.X - 1000.0).abs() >= 1.0 {
            return Ok(());
        }
    }

    Err(From::from("BakkesMod command seemed not to work"))
}

pub trait BakkesModCommand {
    fn to_bakkesmod_command(self) -> String;
}

impl<'a> BakkesModCommand for &'a str {
    fn to_bakkesmod_command(self) -> String {
        self.to_owned()
    }
}

impl BakkesModCommand for String {
    fn to_bakkesmod_command(self) -> String {
        self
    }
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
    /// This is a debug-only convenience function that lets you directly
    /// copy-paste a row from a collect CSV to visualize it.
    #[allow(dead_code)]
    #[deprecated(note = "Use TestScenario::new() instead when writing actual tests.")]
    pub fn from_collected_row(filename: impl AsRef<Path>, time: f32) -> Self {
        let file = File::open(filename).unwrap();
        let mut reader = csv::Reader::from_reader(file);
        for row in reader.records() {
            let row = row.unwrap();
            let row_time: f32 = row[0].parse().unwrap();
            if row_time >= time {
                let snapshot = Snapshot::from_row(row.iter().map(|x| x.parse().unwrap())).unwrap();
                let result = Self {
                    ball_loc: snapshot.ball.loc,
                    ball_rot: snapshot.ball.rot,
                    ball_vel: snapshot.ball.vel,
                    ball_ang_vel: snapshot.ball.ang_vel,
                    car_loc: snapshot.cars[0].loc,
                    car_rot: snapshot.cars[0].rot,
                    car_vel: snapshot.cars[0].vel,
                    car_ang_vel: snapshot.cars[0].ang_vel,
                    enemy_loc: snapshot.cars[1].loc,
                    enemy_rot: snapshot.cars[1].rot,
                    enemy_vel: snapshot.cars[1].vel,
                    enemy_ang_vel: snapshot.cars[1].ang_vel,
                    ..Default::default()
                };
                println!("{}", result.to_source());
                return result;
            }
        }
        panic!("Time not found in recording.");
    }

    /// This is a debug-only convenience function that lets you directly
    /// copy-paste a row from a collect CSV to visualize it.
    #[allow(dead_code)]
    #[deprecated(note = "Use TestScenario::new() instead when writing actual tests.")]
    pub fn from_collect_row(text: &str) -> Self {
        let row = text.split("\t").map(|x| x.parse().unwrap());
        let snapshot = Snapshot::from_row(row).unwrap();
        let car = &snapshot.cars[0];
        let enemy = &snapshot.cars[1];
        Self {
            ball_loc: snapshot.ball.loc,
            ball_rot: snapshot.ball.rot,
            ball_vel: snapshot.ball.vel,
            ball_ang_vel: snapshot.ball.ang_vel,
            car_loc: car.loc,
            car_rot: car.rot,
            car_vel: car.vel,
            car_ang_vel: car.ang_vel,
            enemy_loc: enemy.loc,
            enemy_rot: enemy.rot,
            enemy_vel: enemy.vel,
            enemy_ang_vel: enemy.ang_vel,
            boost: 100,
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

impl BakkesModCommand for TestScenario {
    fn to_bakkesmod_command(self) -> String {
        let commands = [
            format!("ball location {}", serialize_vector(self.ball_loc)),
            // ball rotation is not supported by BakkesMod and is also likely to not matter.
            format!("ball velocity {}", serialize_vector(self.ball_vel)),
            // ball angular_velocity is not supported by BakkesMod and is also likely to not
            // matter.
            format!("player 0 location {}", serialize_vector(self.car_loc)),
            format!("player 0 rotation {}", serialize_rotation(self.car_rot)),
            format!("player 0 velocity {}", serialize_vector(self.car_vel)),
            // player angular_velocity is not supported by BakkesMod and is also likely to not
            // matter.
            format!("player 1 location {}", serialize_vector(self.enemy_loc)),
            format!("player 1 rotation {}", serialize_rotation(self.enemy_rot)),
            format!("player 1 velocity {}", serialize_vector(self.enemy_vel)),
            // player angular_velocity is not supported by BakkesMod and is also likely to not
            // matter.
            format!("boost set {}", self.boost.to_string()),
        ];
        commands.join(";")
    }
}

pub fn serialize_vector(loc: Vector3<f32>) -> String {
    format!("{} {} {}", loc.x, loc.y, loc.z)
}

pub fn serialize_rotation(rot: Rotation3<f32>) -> String {
    /// Convert from radians to URU (Unreal Rotation Units).
    fn to_uru(theta: f32) -> u16 {
        (theta / PI * 32768.0) as u16
    }

    let (pitch, yaw, roll) = rot.to_unreal_angles();
    format!("{} {} {}", to_uru(pitch), to_uru(yaw), to_uru(roll))
}
