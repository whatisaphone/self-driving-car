use collect::ExtendRotation3;
use nalgebra::{Rotation3, Vector2, Vector3};
use rlbot;
use simulate::rl;
pub use utils::fps_counter::FPSCounter;
pub use utils::geometry::{ExtendF32, ExtendVector2, ExtendVector3};

mod fps_counter;
mod geometry;

/// Assuming I am the first car, return the first car.
pub fn my_car(packet: &rlbot::LiveDataPacket) -> &rlbot::PlayerInfo {
    &packet.GameCars[0]
}

/// Assuming the game is a 1v1, return my car and the enemy's car.
pub fn one_v_one(packet: &rlbot::LiveDataPacket) -> (&rlbot::PlayerInfo, &rlbot::PlayerInfo) {
    (&packet.GameCars[0], &packet.GameCars[1])
}

pub fn my_goal_center() -> Vector2<f32> {
    Vector2::new(0.0, -rl::FIELD_MAX_Y)
}

pub fn my_goal_center_2d() -> Vector2<f32> {
    Vector2::new(0.0, -rl::FIELD_MAX_Y)
}

pub fn own_goal_left_post() -> Vector2<f32> {
    Vector2::new(-rl::GOALPOST_X, -rl::FIELD_MAX_Y)
}

pub fn own_goal_right_post() -> Vector2<f32> {
    Vector2::new(rl::GOALPOST_X, -rl::FIELD_MAX_Y)
}

pub fn enemy_goal_center() -> Vector2<f32> {
    Vector2::new(0.0, rl::FIELD_MAX_Y)
}

pub fn enemy_goal_left_post() -> Vector2<f32> {
    Vector2::new(rl::GOALPOST_X, rl::FIELD_MAX_Y)
}

pub fn enemy_goal_right_post() -> Vector2<f32> {
    Vector2::new(-rl::GOALPOST_X, rl::FIELD_MAX_Y)
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
