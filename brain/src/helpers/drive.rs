use crate::{behavior::movement::simple_yaw_diff, utils::geometry::ExtendF32};
use common::prelude::*;
use nalgebra::Point2;
use simulate::Car1D;

pub fn rough_time_drive_to_loc(
    car: &common::halfway_house::PlayerInfo,
    target_loc: Point2<f32>,
) -> f32 {
    let target_dist = (car.Physics.loc_2d() - target_loc).norm();

    let base_time = 2.0 / 120.0 + steer_penalty(car, simple_yaw_diff(&car.Physics, target_loc));

    let mut sim_car = Car1D::new()
        .with_speed(car.Physics.vel().norm())
        .with_boost(car.Boost as f32);
    sim_car.advance_by_distance(target_dist, 1.0, true);

    base_time + sim_car.time()
}

// Very very rough
fn steer_penalty(car: &common::halfway_house::PlayerInfo, desired_aim: f32) -> f32 {
    let turn = (car.Physics.rot().yaw() - desired_aim)
        .normalize_angle()
        .abs();
    // Literally just guessing here
    turn * 0.5
}
