use common::prelude::*;
use nalgebra::{Point2, UnitComplex};

pub fn feasible_hit_angle_toward(
    ball_loc: Point2<f32>,
    car_loc: Point2<f32>,
    ideal_aim: Point2<f32>,
    max_angle_diff: f32,
) -> Point2<f32> {
    let turn = (ball_loc - car_loc).angle_to(&(ideal_aim - ball_loc));
    let adjust = UnitComplex::new(turn.max(-max_angle_diff).min(max_angle_diff));
    ball_loc + adjust * (ball_loc - car_loc)
}

pub fn feasible_hit_angle_away(
    ball_loc: Point2<f32>,
    car_loc: Point2<f32>,
    aim_avoid_loc: Point2<f32>,
    max_angle_adjust: f32,
) -> Point2<f32> {
    let avoid = (ball_loc - car_loc).angle_to(&(aim_avoid_loc - ball_loc));
    let adjust = UnitComplex::new(max_angle_adjust * -avoid.signum());
    ball_loc + adjust * (ball_loc - car_loc)
}
