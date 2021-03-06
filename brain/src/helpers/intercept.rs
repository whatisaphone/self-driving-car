use crate::{helpers::ball::BallFrame, routing::models::CarState};
use common::prelude::*;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use simulate::Car1D;
use std::borrow::Borrow;

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

pub fn naive_ground_intercept_2<BF, IID>(
    start: &CarState,
    ball: impl IntoIterator<Item = BF>,
    predicate: impl Fn(&BallFrame) -> IID,
) -> Option<NaiveIntercept<IID::Data>>
where
    BF: Borrow<BallFrame>,
    IID: IntoInterceptData,
{
    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut sim_car = Car1D::new()
        .with_speed(start.vel.norm())
        .with_boost(start.boost);

    let (sim_ball, data) = ball.into_iter().find_map(|ball| {
        let ball = ball.borrow();

        sim_car.advance(ball.dt(), 1.0, true);

        let target_dist = (ball.loc - start.loc).to_2d().norm() - RADII;
        if sim_car.distance() >= target_dist {
            if let Some(data) = predicate(&ball).into_intercept_data() {
                return Some((ball.clone(), data));
            }
        }

        None
    })?;

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

pub struct NaiveIntercept<D = ()> {
    pub time: f32,
    pub ball_loc: Point3<f32>,
    pub ball_vel: Vector3<f32>,
    pub car_loc: Point3<f32>,
    pub car_speed: f32,
    pub data: D,
}

pub trait IntoInterceptData {
    type Data;
    fn into_intercept_data(self) -> Option<Self::Data>;
}

impl IntoInterceptData for bool {
    type Data = ();

    fn into_intercept_data(self) -> Option<Self::Data> {
        if self {
            Some(())
        } else {
            None
        }
    }
}

impl<T> IntoInterceptData for Option<T> {
    type Data = T;

    fn into_intercept_data(self) -> Option<Self::Data> {
        self
    }
}

/// Very rough estimate of a penalty to `naive_ground_intercept` that accounts
/// for turning. That function does not account for turning, and this tries to
/// patch it up after the fact.
pub fn naive_intercept_penalty(car: &CarState, ball: &BallFrame) -> f32 {
    let ball_loc = ball.loc.to_2d();
    let car_forward = car.forward_axis_2d();
    let car_to_ball = ball_loc - car.loc_2d();

    let fudge = 0.75;
    car_forward.angle_to(&car_to_ball.to_axis()).abs() * fudge
}
