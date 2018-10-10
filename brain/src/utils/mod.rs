use collect::ExtendRotation3;
use nalgebra::{Point3, Rotation3, UnitQuaternion, Vector2, Vector3};
use rlbot;
use simulate::rl;
pub use utils::{
    fps_counter::FPSCounter,
    geometry::{
        ExtendF32, ExtendPoint2, ExtendPoint3, ExtendUnitComplex, ExtendVector2, ExtendVector3,
    },
    iter::TotalF32,
    rlbot_ext::{get_packet_and_inject_rigid_body_tick, physicsify},
    wall_ray_calculator::{Wall, WallRayCalculator},
};

mod fps_counter;
pub mod geometry;
mod iter;
mod rlbot_ext;
mod wall_ray_calculator;

/// Assuming I am the first car, return the first car.
pub fn my_car(packet: &rlbot::ffi::LiveDataPacket) -> &rlbot::ffi::PlayerInfo {
    &packet.GameCars[0]
}

/// Assuming the game is a 1v1, return my car and the enemy's car.
pub fn one_v_one(
    packet: &rlbot::ffi::LiveDataPacket,
) -> (&rlbot::ffi::PlayerInfo, &rlbot::ffi::PlayerInfo) {
    (&packet.GameCars[0], &packet.GameCars[1])
}

pub fn my_goal_center() -> Vector2<f32> {
    Vector2::new(0.0, -rl::FIELD_MAX_Y)
}

pub fn my_goal_center_2d() -> Vector2<f32> {
    Vector2::new(0.0, -rl::FIELD_MAX_Y)
}

pub fn enemy_goal_center() -> Vector2<f32> {
    Vector2::new(0.0, rl::FIELD_MAX_Y)
}

pub trait ExtendPhysics {
    fn loc(&self) -> Vector3<f32>;
    fn locp(&self) -> Point3<f32>;
    fn rot(&self) -> Rotation3<f32>;
    fn quat(&self) -> UnitQuaternion<f32>;
    fn vel(&self) -> Vector3<f32>;
    fn ang_vel(&self) -> Vector3<f32>;
}

impl ExtendPhysics for rlbot::ffi::Physics {
    fn loc(&self) -> Vector3<f32> {
        Vector3::new(self.Location.X, self.Location.Y, self.Location.Z)
    }

    fn locp(&self) -> Point3<f32> {
        Point3::new(self.Location.X, self.Location.Y, self.Location.Z)
    }

    fn rot(&self) -> Rotation3<f32> {
        Rotation3::from_unreal_angles(self.Rotation.Pitch, self.Rotation.Yaw, self.Rotation.Roll)
    }

    fn quat(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_rotation_matrix(&self.rot())
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

pub trait PhysicsState {
    fn loc(&self) -> Point3<f32>;
    fn quat(&self) -> UnitQuaternion<f32>;
    fn vel(&self) -> Vector3<f32>;
}

impl PhysicsState for rlbot::ffi::Physics {
    fn loc(&self) -> Point3<f32> {
        Point3::from_coordinates(ExtendPhysics::loc(self))
    }

    fn quat(&self) -> UnitQuaternion<f32> {
        ExtendPhysics::quat(self)
    }

    fn vel(&self) -> Vector3<f32> {
        ExtendPhysics::vel(self)
    }
}
