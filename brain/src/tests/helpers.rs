use bakkesmod::BakkesMod;
use brain::Brain;
use crossbeam_channel;
use maneuvers::Maneuver;
use nalgebra::{Rotation3, Vector3};
use rlbot;
use std::f32::consts::PI;
use std::thread;
use std::thread::sleep;
use std::time::Duration;
use utils::ExtendRotation3;

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
            ball_loc: Vector3::new(0.0, 0.0, 0.0),
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

pub struct TestRunner {
    terminate: Option<crossbeam_channel::Sender<()>>,
    has_scored: crossbeam_channel::Sender<crossbeam_channel::Sender<bool>>,
    join_handle: Option<thread::JoinHandle<()>>,
}

impl TestRunner {
    pub fn start(maneuver: Box<Maneuver + Send>, scenario: TestScenario) -> TestRunner {
        rlbot::init().unwrap();
        rlbot::start_match(rlbot::match_settings_1v1()).unwrap();

        let mut packets = rlbot::LiveDataPackets::new();

        // Wait for RoundActive
        while !packets.wait().unwrap().GameInfo.RoundActive {}

        setup_scenario(&scenario);

        let (terminate_tx, terminate_rx) = crossbeam_channel::unbounded();
        let (has_scored_tx, has_scored_rx) = crossbeam_channel::bounded(1);
        let thread = thread::spawn(|| test_thread(scenario, maneuver, terminate_rx, has_scored_rx));
        TestRunner {
            terminate: Some(terminate_tx),
            has_scored: has_scored_tx,
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
    pub fn sleep_millis(&self, millis: u64) {
        sleep(Duration::from_millis(millis))
    }

    pub fn has_scored(&self) -> bool {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.has_scored.send(tx);
        rx.recv().unwrap()
    }
}

fn test_thread(
    scenario: TestScenario,
    maneuver: Box<Maneuver>,
    terminate: crossbeam_channel::Receiver<()>,
    has_scored: crossbeam_channel::Receiver<crossbeam_channel::Sender<bool>>,
) {
    let mut brain = Brain::with_maneuver(maneuver);

    let mut packets = rlbot::LiveDataPackets::new();
    let first_packet = packets.wait().unwrap();

    loop {
        if terminate.try_recv().is_some() {
            break;
        }

        let packet = packets.wait().unwrap();
        let input = brain.tick(&packet);
        rlbot::update_player_input(input, 0).unwrap();

        if let Some(chan) = has_scored.try_recv() {
            let first_score = first_packet.match_score();
            let current_score = packet.match_score();
            chan.send(current_score[0] > first_score[0]);
        }
    }

    // For tidiness, make the car stop moving when the test is finished.
    rlbot::update_player_input(Default::default(), 0).unwrap();
}

fn setup_scenario(scenario: &TestScenario) {
    let bakkesmod = BakkesMod::connect().unwrap();
    for _ in 0..10 {
        bakkesmod.send(get_bakkesmod_command_for_scenario(scenario));
        sleep(Duration::from_millis(100));
    }
}

fn get_bakkesmod_command_for_scenario(scenario: &TestScenario) -> String {
    let commands = [
        format!("ball location {}", serialize_vector(scenario.ball_loc)),
        // ball rotation is not supported by BakkesMod and is also likely to not matter.
        format!("ball velocity {}", serialize_vector(scenario.ball_vel)),
        // ball angular_velocity is not supported by BakkesMod and is also likely to not matter.
        format!("player 0 location {}", serialize_vector(scenario.car_loc)),
        format!("player 0 rotation {}", serialize_rotation(scenario.car_rot)),
        format!("player 0 velocity {}", serialize_vector(scenario.car_vel)),
        // player angular_velocity is not supported by BakkesMod and is also likely to not matter.
        format!("player 1 location {}", serialize_vector(scenario.enemy_loc)),
        format!(
            "player 1 rotation {}",
            serialize_rotation(scenario.enemy_rot)
        ),
        format!("player 1 velocity {}", serialize_vector(scenario.enemy_vel)),
        // player angular_velocity is not supported by BakkesMod and is also likely to not matter.
        format!("boost set {}", scenario.boost.to_string()),
    ];
    commands.join(";")
}

pub fn serialize_vector(loc: Vector3<f32>) -> String {
    format!("{} {} {}", loc[0], loc[1], loc[2])
}

pub fn serialize_rotation(rot: Rotation3<f32>) -> String {
    /// Convert from radians to URU (Unreal Rotation Units).
    fn to_uru(theta: f32) -> u16 {
        (theta / PI * 32768.0) as u16
    }

    let (pitch, yaw, roll) = rot.to_unreal_angles();
    format!("{} {} {}", to_uru(pitch), to_uru(yaw), to_uru(roll))
}
