use nalgebra::{Point3, Vector3};
use plan::ball::{BallFrame, BallTrajectory};
use rlbot;
use simulate::Car1D;
use strategy::Context;
use utils::{ExtendPhysics, ExtendVector3};

pub fn estimate_intercept_car_ball(
    ctx: &mut Context,
    car: &rlbot::ffi::PlayerInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Option<Intercept> {
    let intercept = naive_ground_intercept(ctx.scenario.ball_prediction(), car, |bf| {
        predicate(bf.t, &bf.loc.coords, &bf.vel)
    });
    intercept.map(Into::into)
}

pub fn estimate_intercept_car_ball_2(
    ctx: &mut Context,
    car: &rlbot::ffi::PlayerInfo,
    predicate: impl Fn(&BallFrame) -> bool,
) -> Option<NaiveIntercept> {
    naive_ground_intercept(ctx.scenario.ball_prediction(), car, predicate)
}

pub fn naive_ground_intercept(
    ball: &BallTrajectory,
    car: &rlbot::ffi::PlayerInfo,
    predicate: impl Fn(&BallFrame) -> bool,
) -> Option<NaiveIntercept> {
    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut sim_car = Car1D::new(car.Physics.vel().norm()).with_boost(car.Boost);

    let sim_ball = ball.iter().find(|ball| {
        sim_car.step(ball.dt(), 1.0, true);

        let target_traveled = (ball.loc - car.Physics.locp()).to_2d().norm() - RADII;
        if sim_car.distance_traveled() >= target_traveled {
            if predicate(ball) {
                return true;
            }
        }

        false
    });

    let sim_ball = some_or_else!(sim_ball, {
        return None;
    });

    let intercept_loc = sim_ball.loc - (sim_ball.loc - car.Physics.locp()).normalize() * RADII;
    let intercept = NaiveIntercept {
        time: sim_ball.t,
        ball_loc: sim_ball.loc,
        ball_vel: sim_ball.vel,
        car_loc: intercept_loc,
        car_speed: sim_car.speed(),
    };
    Some(intercept)
}

pub struct Intercept {
    pub time: f32,
    pub ball_loc: Vector3<f32>,
    pub ball_vel: Vector3<f32>,
    pub car_loc: Vector3<f32>,
    pub car_speed: f32,
}

pub struct NaiveIntercept {
    pub time: f32,
    pub ball_loc: Point3<f32>,
    pub ball_vel: Vector3<f32>,
    pub car_loc: Point3<f32>,
    pub car_speed: f32,
}

impl From<NaiveIntercept> for Intercept {
    fn from(i: NaiveIntercept) -> Self {
        Self {
            time: i.time,
            ball_loc: i.ball_loc.coords,
            ball_vel: i.ball_vel,
            car_loc: i.car_loc.coords,
            car_speed: i.car_speed,
        }
    }
}
