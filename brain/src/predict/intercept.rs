use nalgebra::Vector3;
use rlbot;
use simulate::{chip::Ball, rl, Car1D};
use utils::ExtendPhysics;

const MAX_SIM_TIME: f32 = 3.0;

pub fn estimate_intercept_car_ball(car: &rlbot::PlayerInfo, ball: &rlbot::BallInfo) -> Intercept {
    estimate_intercept_car_ball_2(car, ball, |_t, loc, _vel| loc.z < 110.0)
}

pub fn estimate_intercept_car_ball_2(
    car: &rlbot::PlayerInfo,
    ball: &rlbot::BallInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Intercept {
    estimate_intercept_car_ball_4(car, ball, predicate).unwrap_or_else(|x| x)
}

pub fn estimate_intercept_car_ball_3(
    car: &rlbot::PlayerInfo,
    ball: &rlbot::BallInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Option<Intercept> {
    estimate_intercept_car_ball_4(car, ball, predicate).ok()
}

pub fn estimate_intercept_car_ball_4(
    car: &rlbot::PlayerInfo,
    ball: &rlbot::BallInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Result<Intercept, Intercept> {
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

    let ok = loop {
        t += DT;
        sim_ball.step(DT);
        sim_car.step(DT, 1.0, true);

        let target_traveled = (sim_ball.loc() - car.Physics.loc()).norm() - RADII;
        if sim_car.distance_traveled() >= target_traveled {
            if predicate(t, &sim_ball.loc(), &sim_ball.vel()) {
                break true;
            }
        }

        if t >= MAX_SIM_TIME {
            break false;
        }
    };

    let intercept_loc = sim_ball.loc() - (sim_ball.loc() - car.Physics.loc()).normalize() * RADII;
    let intercept = Intercept {
        time: t,
        ball_loc: sim_ball.loc(),
        ball_vel: sim_ball.vel(),
        car_loc: intercept_loc,
        car_speed: sim_car.speed(),
    };
    (if ok { Ok } else { Err })(intercept)
}

pub fn is_sane_ball_loc(loc: Vector3<f32>) -> bool {
    loc.x.abs() < rl::FIELD_MAX_X && loc.y.abs() < rl::FIELD_MAX_Y
}

pub struct Intercept {
    pub time: f32,
    pub ball_loc: Vector3<f32>,
    pub ball_vel: Vector3<f32>,
    pub car_loc: Vector3<f32>,
    pub car_speed: f32,
}
