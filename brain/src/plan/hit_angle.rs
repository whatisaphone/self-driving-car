use nalgebra::Vector2;
use utils::{ExtendF32, ExtendVector2};

pub fn feasible_hit_angle(
    ball_loc: Vector2<f32>,
    car_loc: Vector2<f32>,
    ideal_aim: Vector2<f32>,
    max_angle_diff: f32,
) -> f32 {
    let angle_car_ball = car_loc.angle_to(ball_loc);
    let angle_ball_ideal = ball_loc.angle_to(ideal_aim);
    let correction = (angle_ball_ideal - angle_car_ball).normalize_angle();
    angle_car_ball + correction.max(-max_angle_diff).min(max_angle_diff)
}
