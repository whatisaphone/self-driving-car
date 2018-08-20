use nalgebra::{Rotation3, Vector3};
use rlbot;

pub mod fps_counter;

/// Assuming I am the first car, return the first car.
pub fn my_car(packet: &rlbot::LiveDataPacket) -> rlbot::PlayerInfo {
    packet.GameCars[0]
}

/// Assuming the game is a 1v1, return my car and the enemy's car.
pub fn one_v_one(packet: &rlbot::LiveDataPacket) -> (rlbot::PlayerInfo, rlbot::PlayerInfo) {
    (packet.GameCars[0], packet.GameCars[1])
}

pub trait ExtendPhysics {
    fn loc(&self) -> Vector3<f32>;
    fn rot(&self) -> Rotation3<f32>;
    fn vel(&self) -> Vector3<f32>;
    fn ang_vel(&self) -> Vector3<f32>;
}

impl ExtendPhysics for rlbot::Physics {
    fn loc(&self) -> Vector3<f32> {
        Vector3::new(self.Location.X, self.Location.Y, self.Location.Z)
    }

    fn rot(&self) -> Rotation3<f32> {
        Rotation3::from_unreal_angles(self.Rotation.Pitch, self.Rotation.Yaw, self.Rotation.Roll)
    }

    fn vel(&self) -> Vector3<f32> {
        Vector3::new(self.Velocity.X, self.Velocity.Y, self.Velocity.Z)
    }

    fn ang_vel(&self) -> Vector3<f32> {
        Vector3::new(
            self.AngularVelocity.X,
            self.AngularVelocity.Y,
            self.AngularVelocity.Z,
        )
    }
}

pub trait ExtendRotation3 {
    fn from_unreal_angles(pitch: f32, yaw: f32, roll: f32) -> Rotation3<f32>;
    fn to_unreal_angles(&self) -> (f32, f32, f32);
    fn pitch(&self) -> f32;
    fn yaw(&self) -> f32;
    fn roll(&self) -> f32;
}

// There are three different rotation conventions we need to deal with:
//
// 1. `Rotation3::from_euler_angles` – (roll, pitch, yaw).
// 2. `Rotation3::to_euler_angles` – (roll, yaw, pitch).
// 3. Unreal itself – (yaw, pitch, roll).
//
// That is not tenable, so I reimplemented some things using Unreal's rotation
// order.
impl ExtendRotation3 for Rotation3<f32> {
    fn from_unreal_angles(pitch: f32, yaw: f32, roll: f32) -> Rotation3<f32> {
        Rotation3::from_euler_angles(roll, pitch, yaw)
    }

    fn to_unreal_angles(&self) -> (f32, f32, f32) {
        let (roll, pitch, yaw) = self.to_euler_angles();
        (pitch, yaw, roll)
    }

    fn pitch(&self) -> f32 {
        let (pitch, _yaw, _roll) = self.to_unreal_angles();
        pitch
    }

    fn yaw(&self) -> f32 {
        let (_pitch, yaw, _roll) = self.to_unreal_angles();
        yaw
    }

    fn roll(&self) -> f32 {
        let (_pitch, _yaw, roll) = self.to_unreal_angles();
        roll
    }
}
