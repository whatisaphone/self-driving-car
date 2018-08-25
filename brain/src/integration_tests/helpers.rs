use bakkesmod::BakkesMod;
use behavior::Behavior;
use brain::Brain;
use collect::{ExtendRotation3, Snapshot};
use crossbeam_channel;
use nalgebra::{Rotation3, Vector3};
use rlbot;
use std::f32::consts::PI;
use std::sync::{Arc, Barrier, Mutex, MutexGuard};
use std::thread;
use std::thread::sleep;
use std::time::Duration;

pub struct TestRunner {
    sniff_packet: crossbeam_channel::Sender<crossbeam_channel::Sender<rlbot::LiveDataPacket>>,
    has_scored: crossbeam_channel::Sender<crossbeam_channel::Sender<bool>>,
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

    pub fn start2<B, BF>(
        scenario: impl BakkesModCommand + Send + 'static,
        behavior: BF,
    ) -> TestRunner
    where
        B: Behavior + 'static,
        BF: FnOnce(&rlbot::LiveDataPacket) -> B + Send + 'static,
    {
        let ready_wait = Arc::new(Barrier::new(2));
        let ready_wait_send = ready_wait.clone();
        let (terminate_tx, terminate_rx) = crossbeam_channel::unbounded();
        let (sniff_packet_tx, sniff_packet_rx) = crossbeam_channel::unbounded();
        let (has_scored_tx, has_scored_rx) = crossbeam_channel::unbounded();
        let thread = thread::spawn(|| {
            test_thread(
                scenario,
                |p| Box::new(behavior(p)),
                ready_wait_send,
                sniff_packet_rx,
                has_scored_rx,
                terminate_rx,
            )
        });

        ready_wait.wait();
        TestRunner {
            sniff_packet: sniff_packet_tx,
            has_scored: has_scored_tx,
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

    pub fn sniff_packet(&self) -> rlbot::LiveDataPacket {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sniff_packet.send(tx);
        rx.recv().unwrap()
    }

    pub fn has_scored(&self) -> bool {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.has_scored.send(tx);
        rx.recv().unwrap()
    }
}

lazy_static! {
    static ref RLBOT_MUTEX: Mutex<Option<rlbot::RLBot>> = Mutex::new(None);
}

/// RLBot_Core_Interface.dll sometimes crashes if it's unloaded and then
/// reloaded, so just keep a permanent instance around for the tests (and
/// leak it, don't worry, everything will be fine!)
pub fn unlock_rlbot_singleton() -> MutexGuard<'static, Option<rlbot::RLBot>> {
    let mut rlbot_guard = RLBOT_MUTEX.lock().unwrap();
    if rlbot_guard.is_none() {
        *rlbot_guard = Some(rlbot::init().unwrap());
    }
    rlbot_guard
}

fn test_thread(
    scenario: impl BakkesModCommand,
    behavior: impl FnOnce(&rlbot::LiveDataPacket) -> Box<Behavior>,
    ready_wait: Arc<Barrier>,
    sniff_packet: crossbeam_channel::Receiver<crossbeam_channel::Sender<rlbot::LiveDataPacket>>,
    has_scored: crossbeam_channel::Receiver<crossbeam_channel::Sender<bool>>,
    terminate: crossbeam_channel::Receiver<()>,
) {
    let rlbot_guard = unlock_rlbot_singleton();
    let rlbot = rlbot_guard.as_ref().unwrap();
    rlbot.start_match(rlbot::match_settings_1v1()).unwrap();

    let mut packets = rlbot.packeteer();

    // Wait for RoundActive
    while !packets.next().unwrap().GameInfo.RoundActive {}

    setup_scenario(scenario);
    ready_wait.wait();

    let first_packet = packets.next().unwrap();

    let mut brain = Brain::with_behavior(behavior(&first_packet));

    loop {
        let packet = packets.next().unwrap();
        let input = brain.tick(&packet);
        rlbot.update_player_input(input, 0).unwrap();

        if let Some(chan) = has_scored.try_recv() {
            let first_score = first_packet.match_score();
            let current_score = packet.match_score();
            chan.send(current_score[0] > first_score[0]); // Index 0 means blue team
        }

        if let Some(chan) = sniff_packet.try_recv() {
            chan.send(packet);
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
    let command = scenario.to_bakkesmod_command();
    for _ in 0..6 {
        bakkesmod.send(command.to_owned());
        sleep(Duration::from_millis(250));
    }
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
            car_loc: Vector3::new(0.0, 0.0, 0.0),
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
