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
