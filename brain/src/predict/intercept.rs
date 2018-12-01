use common::prelude::*;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use plan::ball::BallFrame;
use rlbot;
use routing::models::CarState;
use simulate::Car1Dv2;
use std::borrow::Borrow;
use strategy::Context;

pub fn estimate_intercept_car_ball(
    ctx: &Context,
    car: &rlbot::ffi::PlayerInfo,
    predicate: impl Fn(f32, &Point3<f32>, &Vector3<f32>) -> bool,
) -> Option<Intercept> {
    let intercept = naive_ground_intercept(
        ctx.scenario.ball_prediction().iter(),
        car.Physics.locp(),
        car.Physics.vel(),
        car.Boost as f32,
        |bf| predicate(bf.t, &bf.loc, &bf.vel),
    );
    intercept.map(Into::into)
}

pub fn naive_ground_intercept<'a>(
    ball: impl Iterator<Item = &'a BallFrame>,
    start_loc: Point3<f32>,
    start_vel: Vector3<f32>,
    start_boost: f32,
    predicate: impl Fn(&BallFrame) -> bool,
) -> Option<NaiveIntercept> {
    let start = CarState {
        loc: start_loc,
        rot: UnitQuaternion::identity(), // This isn't used.
        vel: start_vel,
        boost: start_boost,
    };
    naive_ground_intercept_2(
        &start,
        ball,
        |ball| if predicate(ball) { Some(()) } else { None },
    )
}

pub fn naive_ground_intercept_2<T, BF: Borrow<BallFrame>>(
    start: &CarState,
    ball: impl IntoIterator<Item = BF>,
    predicate: impl Fn(&BallFrame) -> Option<T>,
) -> Option<NaiveIntercept<T>> {
    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut sim_car = Car1Dv2::new()
        .with_speed(start.vel.norm())
        .with_boost(start.boost);

    let sim_ball = ball.into_iter().find_map(|ball| {
        let ball = ball.borrow();

        sim_car.advance(ball.dt(), 1.0, true);

        let target_dist = (ball.loc - start.loc).to_2d().norm() - RADII;
        if sim_car.distance() >= target_dist {
            if let Some(data) = predicate(&ball) {
                return Some((ball.clone(), data));
            }
        }

        None
    });

    let (sim_ball, data) = some_or_else!(sim_ball, {
        return None;
    });

    let intercept_loc = sim_ball.loc - (sim_ball.loc - start.loc).normalize() * RADII;
    let intercept = NaiveIntercept {
        time: sim_ball.t,
        ball_loc: sim_ball.loc,
        ball_vel: sim_ball.vel,
        car_loc: intercept_loc,
        car_speed: sim_car.speed(),
        data,
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

pub struct NaiveIntercept<T = ()> {
    pub time: f32,
    pub ball_loc: Point3<f32>,
    pub ball_vel: Vector3<f32>,
    pub car_loc: Point3<f32>,
    pub car_speed: f32,
    pub data: T,
}

impl<T> From<NaiveIntercept<T>> for Intercept {
    fn from(i: NaiveIntercept<T>) -> Self {
        Self {
            time: i.time,
            ball_loc: i.ball_loc.coords,
            ball_vel: i.ball_vel,
            car_loc: i.car_loc.coords,
            car_speed: i.car_speed,
        }
    }
}
