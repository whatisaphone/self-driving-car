use nalgebra::Vector3;
use rlbot;
use simulate::{Ball, Car1D};
use utils::ExtendPhysics;

pub fn estimate_intercept_car_ball(
    car: &rlbot::PlayerInfo,
    ball: &rlbot::BallInfo,
) -> InterceptResult {
    estimate_intercept_car_ball_2(car, ball, |loc, _vel| loc.z < 110.0)
}

pub fn estimate_intercept_car_ball_2(
    car: &rlbot::PlayerInfo,
    ball: &rlbot::BallInfo,
    predicate: impl FnOnce(&Vector3<f32>, &Vector3<f32>) -> bool,
) -> InterceptResult {
    const DT: f32 = 1.0 / 60.0;

    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut t = 0.0;
    let mut sim_car = Car1D::new(car.Physics.vel().norm()).with_boost(car.Boost);
    let mut sim_ball = Ball::new(
        ball.Physics.loc(),
        ball.Physics.vel(),
        ball.Physics.ang_vel(),
    );

    for _ in 0..200 {
        t += DT;
        sim_ball.step(DT);
        sim_car.step(DT, 1.0, true);

        if sim_ball.loc().z > 110.0 {
            continue; // The ball is so high and I don't know how to jump :(
        }

        let target_traveled = (sim_ball.loc() - car.Physics.loc()).norm() - RADII;
        if sim_car.distance_traveled() >= target_traveled {
            break;
        }
    }

    let intercept_loc = sim_ball.loc() - (sim_ball.loc() - car.Physics.loc()).normalize() * RADII;
    InterceptResult {
        time: t,
        ball_loc: sim_ball.loc(),
        car_loc: intercept_loc,
    }
}

pub struct InterceptResult {
    pub time: f32,
    pub ball_loc: Vector3<f32>,
    pub car_loc: Vector3<f32>,
}
