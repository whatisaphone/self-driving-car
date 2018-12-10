use crate::{
    behavior::Behavior,
    mechanics::{simple_yaw_diff, QuickJumpAndDodge},
    utils::geometry::ExtendF32,
};
use common::{prelude::*, rl};
use nalgebra::Point2;
use rlbot;
use simulate::Car1Dv2;
use std::f32::consts::PI;

// I'm keeping this value artificially high until I implement smarter routing.
const GROUND_DODGE_TIME: f32 = 2.0;

pub fn rough_time_drive_to_loc(car: &rlbot::ffi::PlayerInfo, target_loc: Point2<f32>) -> f32 {
    let target_dist = (car.Physics.loc().to_2d() - target_loc).norm();

    let base_time = 2.0 / 120.0 + steer_penalty(car, simple_yaw_diff(&car.Physics, target_loc));

    let mut sim_car = Car1Dv2::new()
        .with_speed(car.Physics.vel().norm())
        .with_boost(car.Boost as f32);
    sim_car.advance_by_distance(target_dist, 1.0, true);

    base_time + sim_car.time()
}

// Very very rough
fn steer_penalty(car: &rlbot::ffi::PlayerInfo, desired_aim: f32) -> f32 {
    let turn = (car.Physics.rot().yaw() - desired_aim)
        .normalize_angle()
        .abs();
    // Literally just guessing here
    turn * 3.0 / 4.0
}

pub fn get_route_dodge(
    car: &rlbot::ffi::PlayerInfo,
    target_loc: Point2<f32>,
) -> Option<Box<Behavior>> {
    const DODGE_SPEED_BOOST: f32 = 500.0; // TODO: Literally just guessed this

    if !car.OnGround {
        return None;
    }
    if car.Physics.rot().pitch() >= PI / 12.0 {
        return None;
    }

    if simple_yaw_diff(&car.Physics, target_loc).abs() >= PI / 60.0 {
        return None;
    }

    if car.Physics.vel().norm() < 1300.0 {
        // This number is just a total guess
        return None; // It's faster to accelerate.
    }
    if car.Physics.vel().norm() >= rl::CAR_ALMOST_MAX_SPEED {
        return None; // We can't get any faster.
    }

    let target_dist = (car.Physics.loc().to_2d() - target_loc).norm();
    let dodge_vel = car.Physics.vel().norm() + DODGE_SPEED_BOOST;
    let travel_time = target_dist / dodge_vel;
    if travel_time < GROUND_DODGE_TIME {
        return None;
    }

    Some(Box::new(QuickJumpAndDodge::new()))
}
