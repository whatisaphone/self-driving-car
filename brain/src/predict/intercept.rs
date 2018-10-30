use common::prelude::*;
use nalgebra::{Point3, Vector3};
use plan::ball::BallFrame;
use rlbot;
use simulate::Car1D;
use strategy::Context;
use utils::ExtendVector3;

pub fn estimate_intercept_car_ball(
    ctx: &mut Context,
    car: &rlbot::ffi::PlayerInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Option<Intercept> {
    let intercept = naive_ground_intercept(
        ctx.scenario.ball_prediction().iter(),
        car.Physics.locp(),
        car.Physics.vel(),
        car.Boost as f32,
        |bf| predicate(bf.t, &bf.loc.coords, &bf.vel),
    );
    intercept.map(Into::into)
}

pub fn naive_ground_intercept<'a>(
    mut ball: impl Iterator<Item = &'a BallFrame>,
    start_loc: Point3<f32>,
    start_vel: Vector3<f32>,
    start_boost: f32,
    predicate: impl Fn(&BallFrame) -> bool,
) -> Option<NaiveIntercept> {
    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut sim_car = Car1D::new(start_vel.norm()).with_boost_float(start_boost);

    let sim_ball = ball.find(|ball| {
        sim_car.step(ball.dt(), 1.0, true);

        let target_traveled = (ball.loc - start_loc).to_2d().norm() - RADII;
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

    let intercept_loc = sim_ball.loc - (sim_ball.loc - start_loc).normalize() * RADII;
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
