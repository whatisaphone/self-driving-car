use collect::{RecordingRigidBodyState, RecordingTick};
use common::prelude::*;
use nalgebra::{Point3, Rotation3, UnitQuaternion, Vector3};
use std::{f32::consts::PI, fs::File, path::Path};

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

    pub fn ball(&self) -> RecordingRigidBodyState {
        RecordingRigidBodyState {
            loc: Point3::from(self.ball_loc),
            rot: UnitQuaternion::from_rotation_matrix(&self.ball_rot),
            vel: self.ball_vel,
            ang_vel: self.ball_ang_vel,
        }
    }

    pub fn car(&self) -> RecordingRigidBodyState {
        RecordingRigidBodyState {
            loc: Point3::from(self.car_loc),
            rot: UnitQuaternion::from_rotation_matrix(&self.car_rot),
            vel: self.car_vel,
            ang_vel: self.car_ang_vel,
        }
    }

    pub fn enemy(&self) -> RecordingRigidBodyState {
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
            ball_rot: Rotation3::from_unreal_angles({}, {}, {}),
            ball_vel: Vector3::new({}, {}, {}),
            ball_ang_vel: Vector3::new({}, {}, {}),
            car_loc: Point3::new({}, {}, {}),
            car_rot: Rotation3::from_unreal_angles({}, {}, {}),
            car_vel: Vector3::new({}, {}, {}),
            car_ang_vel: Vector3::new({}, {}, {}),
            enemy_loc: Point3::new({}, {}, {}),
            enemy_rot: Rotation3::from_unreal_angles({}, {}, {}),
            enemy_vel: Vector3::new({}, {}, {}),
            enemy_ang_vel: Vector3::new({}, {}, {}),
            boost: {},
        }}",
            self.ball_loc.x,
            self.ball_loc.y,
            self.ball_loc.z,
            self.ball_rot.pitch(),
            self.ball_rot.yaw(),
            self.ball_rot.roll(),
            self.ball_vel.x,
            self.ball_vel.y,
            self.ball_vel.z,
            self.ball_ang_vel.x,
            self.ball_ang_vel.y,
            self.ball_ang_vel.z,
            self.car_loc.x,
            self.car_loc.y,
            self.car_loc.z,
            self.car_rot.pitch(),
            self.car_rot.yaw(),
            self.car_rot.roll(),
            self.car_vel.x,
            self.car_vel.y,
            self.car_vel.z,
            self.car_ang_vel.x,
            self.car_ang_vel.y,
            self.car_ang_vel.z,
            self.enemy_loc.x,
            self.enemy_loc.y,
            self.enemy_loc.z,
            self.enemy_rot.pitch(),
            self.enemy_rot.yaw(),
            self.enemy_rot.roll(),
            self.enemy_vel.x,
            self.enemy_vel.y,
            self.enemy_vel.z,
            self.enemy_ang_vel.x,
            self.enemy_ang_vel.y,
            self.enemy_ang_vel.z,
            self.boost
        )
    }
}
