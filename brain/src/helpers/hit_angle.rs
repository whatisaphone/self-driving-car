use crate::utils::geometry::ExtendF32;
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

/// Rotate `mobile` around `center` as far as possible towards `ideal` without
/// exceeding `max_angle_diff`.
pub fn feasible_angle_near(
    center: Point2<f32>,
    mobile: Point2<f32>,
    ideal: Point2<f32>,
    max_angle_diff: f32,
) -> Point2<f32> {
    let turn = (mobile - center).angle_to(&(ideal - center));
    let adjust = UnitComplex::new(turn.max(-max_angle_diff).min(max_angle_diff));
    center + adjust * (mobile - center)
}

/// Calculate an angle from `ball_loc` to `car_loc`, trying to get between
/// `ball_loc` and `block_loc`, but not adjusting the approach angle by more
/// than `max_angle_diff`.
pub fn blocking_angle(
    ball_loc: Point2<f32>,
    car_loc: Point2<f32>,
    block_loc: Point2<f32>,
    max_angle_diff: f32,
) -> f32 {
    let naive_angle = ball_loc.negated_difference_and_angle_to(car_loc);
    let block_angle = ball_loc.negated_difference_and_angle_to(block_loc);
    let adjust = (block_angle - naive_angle)
        .normalize_angle()
        .max(-max_angle_diff)
        .min(max_angle_diff);
    (naive_angle + adjust).normalize_angle()
}
