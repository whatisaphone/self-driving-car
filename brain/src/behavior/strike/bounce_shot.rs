use crate::{
    predict::intercept::NaiveIntercept,
    strategy::Goal,
    utils::{geometry::ExtendF32, WallRayCalculator},
};
use common::prelude::*;
use nalgebra::Point2;
use std::f32::consts::PI;

pub struct BounceShot {
    _private: (),
}

impl BounceShot {
    /// Given a ball location, where should we aim the shot?
    pub fn aim_loc(goal: &Goal, car_loc: Point2<f32>, ball_loc: Point2<f32>) -> Point2<f32> {
        // If the ball is very close to goal, aim for a point in goal opposite from the
        // ball for an easy shot. If there's some distance, aim at the middle of goal
        // so we're less likely to miss.
        let y_dist = (goal.center_2d.y - ball_loc.y).abs();
        let allow_angle_diff = ((1000.0 - y_dist) / 1000.0).max(0.0) * PI / 12.0;
        let naive_angle = car_loc.coords.angle_to(ball_loc.coords);
        let goal_angle = ball_loc.coords.angle_to(goal.center_2d.coords);
        let adjust = (naive_angle - goal_angle).normalize_angle();
        let aim_angle = goal_angle + adjust.max(-allow_angle_diff).min(allow_angle_diff);
        WallRayCalculator::calc_ray(ball_loc, aim_angle)
    }

    /// Roughly where should the car be when it makes contact with the ball, in
    /// order to shoot at `aim_loc`?
    pub fn rough_shooting_spot(intercept: &NaiveIntercept, aim_loc: Point2<f32>) -> Point2<f32> {
        // This is not the greatest guess
        let guess_final_ball_speed = 1700.0;
        let ball_to_aim = aim_loc - intercept.ball_loc.to_2d();
        if ball_to_aim.norm() < 0.1 {
            log::warn!("[rough_shooting_spot] ball_loc == aim_loc; bailing");
            // This happened in a Dropshot game when WallRayCalculator was still using the
            // standard Soccar mesh. It's a degenerate case, but we at least shouldn't start
            // spewing NaN everywhere.
            return intercept.car_loc.to_2d();
        }
        let desired_vel = ball_to_aim.normalize() * guess_final_ball_speed;
        let intercept_vel = intercept.ball_vel.to_2d();
        let impulse = desired_vel - intercept_vel;
        intercept.ball_loc.to_2d() - impulse.normalize() * 200.0
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::helpers::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    // `Repeat` is used in these tests so the shot is not aborted by
    // `SameBallTrajectory` when the ball bounces.

    #[test]
    #[ignore(note = "TODO")]
    fn normal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2000.0, 2000.0, 500.0),
                ball_vel: Vector3::new(1000.0, 0.0, 0.0),
                car_loc: Point3::new(0.0, 0.0, 17.01),
                car_vel: Vector3::new(0.0, 0.0, 0.0),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "TODO")]
    fn slow_no_boost() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2000.0, 2000.0, 1000.0),
                ball_vel: Vector3::new(500.0, 0.0, 0.0),
                car_loc: Point3::new(0.0, 0.0, 17.01),
                car_vel: Vector3::new(0.0, 0.0, 0.0),
                boost: 0,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(6000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "TODO")]
    fn face_target_before_estimating_approach() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(866.92804, -4290.7188, 353.78827),
                ball_vel: Vector3::new(-166.86324, -8.325447, 345.70105),
                car_loc: Point3::new(1816.7043, -4648.5, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.3079103, 0.0),
                car_vel: Vector3::new(30.373384, 216.24547, 8.311),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().norm() >= 1000.0);
    }

    #[test]
    #[ignore(note = "TODO")]
    fn long_high_bouncing_save() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(90.25211, -340.07803, 1487.03),
                ball_vel: Vector3::new(27.551777, -1300.1466, -571.16125),
                car_loc: Point3::new(-636.6111, 538.8031, 16.544558),
                car_rot: Rotation3::from_unreal_angles(-0.01236772, -1.6032016, 0.0000958738),
                car_vel: Vector3::new(-60.050007, -1915.0122, 15.930969),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(2500);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.loc().x >= 1000.0);
    }
}
