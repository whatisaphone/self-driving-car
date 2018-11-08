use common::rl;
use nalgebra::{Point2, Vector2};
use rlbot;
pub use utils::{
    fps_counter::FPSCounter,
    iter::TotalF32,
    wall_ray_calculator::{Wall, WallRayCalculator},
};

mod fps_counter;
pub mod geometry;
mod iter;
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
    enemy_goal_center_point().coords
}

pub fn enemy_goal_center_point() -> Point2<f32> {
    Point2::new(0.0, rl::FIELD_MAX_Y)
}
