use behavior::{Action, Behavior};
use eeg::{color, Drawable};
use mechanics::{simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use nalgebra::Vector2;
use predict::{estimate_intercept_car_ball, Intercept};
use rules::SameBallTrajectory;
use simulate::rl;
use std::f32::consts::PI;
use strategy::Context;
use utils::{
    enemy_goal_center, ExtendF32, ExtendPhysics, ExtendVector2, ExtendVector3, WallRayCalculator,
};

pub struct BounceShot {
    aim_loc: Vector2<f32>,
    same_ball_trajectory: SameBallTrajectory,
}

impl BounceShot {
    pub const MAX_BALL_Z: f32 = 110.0;

    pub fn new() -> Self {
        Self {
            aim_loc: enemy_goal_center(),
            same_ball_trajectory: SameBallTrajectory::new(),
        }
    }

    pub fn with_target_loc(self, aim_loc: Vector2<f32>) -> Self {
        Self { aim_loc, ..self }
    }
}

impl Behavior for BounceShot {
    fn name(&self) -> &str {
        stringify!(BounceShot)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        return_some!(self.same_ball_trajectory.execute(ctx));

        let me = ctx.me();
        let intercept = estimate_intercept_car_ball(ctx, me, |_t, loc, vel| {
            // What we actually want is vel.z >= 0, e.g. the upward half of a bounce. But
            // velocity will be approx. -6.8 when the ball is stationary, due to gravity
            // being applied after collision handling.
            loc.z < Self::MAX_BALL_Z && vel.z >= -10.0
        });

        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[BounceShot] unknown intercept");
            return Action::Abort;
        });

        let intercept_car_loc = Self::rough_shooting_spot(&intercept, self.aim_loc);
        let distance = (ctx.me().Physics.loc().to_2d() - intercept_car_loc).norm();

        ctx.eeg.draw(Drawable::Crosshair(self.aim_loc));
        ctx.eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        ctx.eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));

        if intercept.time < QuickJumpAndDodge::MIN_DODGE_TIME {
            return self.flip(ctx);
        }

        // TODO: this is not how this works…
        let mut child = GroundAccelToLoc::new(
            intercept_car_loc,
            ctx.packet.GameInfo.TimeSeconds + intercept.time,
        );
        child.execute2(ctx)
    }
}

impl BounceShot {
    /// Given a ball location, where should we aim the shot?
    pub fn aim_loc(car_loc: Vector2<f32>, ball_loc: Vector2<f32>) -> Vector2<f32> {
        // If the ball is very close to goal, aim for a point in goal opposite from the
        // ball for an easy shot. If there's some distance, aim at the middle of goal
        // so we're less likely to miss.
        let y_dist = (enemy_goal_center().y - ball_loc.y).abs();
        let allow_angle_diff = ((1000.0 - y_dist) / 1000.0).max(0.0) * PI / 12.0;
        let naive_angle = car_loc.angle_to(ball_loc);
        let goal_angle = ball_loc.angle_to(enemy_goal_center());
        let adjust = (naive_angle - goal_angle).normalize_angle();
        let aim_angle = goal_angle + adjust.max(-allow_angle_diff).min(allow_angle_diff);
        WallRayCalculator::calc_ray(ball_loc, aim_angle)
    }

    /// Roughly where should the car be when it makes contact with the ball, in
    /// order to shoot at `aim_loc`?
    pub fn rough_shooting_spot(intercept: &Intercept, aim_loc: Vector2<f32>) -> Vector2<f32> {
        // This is not the greatest guess
        let guess_final_ball_speed = f32::min(intercept.car_speed * 1.25, rl::CAR_MAX_SPEED);
        let desired_vel =
            (aim_loc - intercept.ball_loc.to_2d()).normalize() * guess_final_ball_speed;
        let intercept_vel = intercept.ball_vel.to_2d();
        let impulse = desired_vel - intercept_vel;
        intercept.ball_loc.to_2d() - impulse.normalize() * 200.0
    }

    fn flip(&mut self, ctx: &mut Context) -> Action {
        let angle = simple_yaw_diff(&ctx.me().Physics, ctx.packet.GameBall.Physics.loc().to_2d());
        Action::call(QuickJumpAndDodge::begin(ctx.packet).angle(angle))
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::Repeat;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::bounce_shot::BounceShot;
    use nalgebra::{Rotation3, Vector2, Vector3};
    use simulate::rl;
    use utils::ExtendPhysics;

    // `Repeat` is used in these tests so the shot is not aborted by
    // `SameBallTrajectory` when the ball bounces.

    #[test]
    fn normal() {
        let test = TestRunner::start(
            Repeat::new(BounceShot::new),
            TestScenario {
                ball_loc: Vector3::new(-2000.0, 2000.0, 500.0),
                ball_vel: Vector3::new(1000.0, 0.0, 0.0),
                car_loc: Vector3::new(0.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, -200.0, 0.0),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn slow_no_boost() {
        let test = TestRunner::start(
            Repeat::new(BounceShot::new),
            TestScenario {
                ball_loc: Vector3::new(-2000.0, 2000.0, 1000.0),
                ball_vel: Vector3::new(500.0, 0.0, 0.0),
                car_loc: Vector3::new(0.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, -200.0, 0.0),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(6000);

        assert!(test.has_scored());
    }

    #[test]
    fn face_target_before_estimating_approach() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(866.92804, -4290.7188, 353.78827),
            ball_vel: Vector3::new(-166.86324, -8.325447, 345.70105),
            car_loc: Vector3::new(1816.7043, -4648.5, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.3079103, 0.0),
            car_vel: Vector3::new(30.373384, 216.24547, 8.311),
            ..Default::default()
        });
        test.set_behavior(Repeat::new(|| {
            BounceShot::new().with_target_loc(Vector2::new(-rl::FIELD_MAX_X, -1000.0))
        }));

        test.sleep_millis(3000);
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().norm() >= 1000.0);
    }
}
