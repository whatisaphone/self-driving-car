use collect::ExtendRotation3;
use mechanics::simple_yaw_diff;
use nalgebra::Vector2;
use rlbot;
use simulate::Car1D;
use utils::{ExtendF32, ExtendPhysics, ExtendVector3};

pub fn rough_time_drive_to_loc(car: &rlbot::PlayerInfo, target_loc: Vector2<f32>) -> f32 {
    const DT: f32 = 1.0 / 60.0;

    let target_dist = (car.Physics.loc().to_2d() - target_loc).norm();

    let mut t = 2.0 / 120.0 + steer_penalty(car, simple_yaw_diff(&car.Physics, target_loc));
    let mut sim_car = Car1D::new(car.Physics.vel().norm()).with_boost(car.Boost);
    loop {
        t += DT;
        sim_car.step(DT, 1.0, true);

        if sim_car.distance_traveled() >= target_dist {
            break;
        }
    }
    t
}

// Very very rough
fn steer_penalty(car: &rlbot::PlayerInfo, desired_aim: f32) -> f32 {
    let turn = (car.Physics.rot().yaw() - desired_aim)
        .normalize_angle()
        .abs();
    // Literally just guessing here
    turn * 3.0 / 4.0
}
