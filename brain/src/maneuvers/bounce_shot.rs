use behavior::{Action, Behavior};
use eeg::{color, Drawable};
use mechanics::{simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use ncollide3d::{
    query::distance,
    shape::{Ball, Triangle},
};
use predict::{estimate_intercept_car_ball_2, Intercept};
use rules::SameBallTrajectory;
use simulate::rl;
use strategy::Context;
use utils::{enemy_goal_center, one_v_one, ExtendPhysics, ExtendVector2, ExtendVector3};

pub struct BounceShot {
    aim_loc: Vector2<f32>,
    same_ball_trajectory: SameBallTrajectory,
}

impl BounceShot {
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
    fn name(&self) -> &'static str {
        stringify!(BounceShot)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        return_some!(self.same_ball_trajectory.execute(ctx));

        let (me, _enemy) = one_v_one(ctx.packet);
        let intercept = estimate_intercept_car_ball_2(&me, &ctx.packet.GameBall, |_t, loc, vel| {
            // What we actually want is vel.z >= 0, e.g. the upward half of a bounce. But
            // velocity will be approx. -6.8 when the ball is stationary, due to gravity
            // being applied after collision handling.
            loc.z < 110.0 && vel.z >= -10.0
        });

        let intercept_car_loc = Self::rough_shooting_spot(&intercept, self.aim_loc);
        let distance = (me.Physics.loc().to_2d() - intercept_car_loc).norm();

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

        // TODO the threshold
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
    pub fn aim_loc(ball_loc: Vector2<f32>) -> Vector2<f32> {
        if ball_loc.x.abs() >= rl::GOALPOST_X {
            return enemy_goal_center();
        }

        // If the ball is very close to net – aim straight in for an easy shot.
        // If there's some distance – aim to the middle of net so we're less likely to
        // miss.
        //
        // This is a pretty rough concept, but it's better than nothing.

        let dist = distance(
            &Isometry3::new(Vector3::zeros(), Vector3::zeros()),
            &Triangle::new(
                Point3::from_coordinates(Vector3::new(-rl::GOALPOST_X, rl::FIELD_MAX_Y, 0.0)),
                Point3::from_coordinates(Vector3::new(rl::GOALPOST_X, rl::FIELD_MAX_Y, 0.0)),
                Point3::from_coordinates(Vector3::new(0.0, rl::FIELD_MAX_Y - rl::GOALPOST_X, 0.0)),
            ),
            &Isometry3::new(ball_loc.to_3d(0.0), Vector3::zeros()),
            &Ball::new(rl::BALL_RADIUS),
        );

        Vector2::new(
            (ball_loc.x.abs() - dist).max(0.0) * ball_loc.x.signum(),
            rl::FIELD_MAX_Y,
        )
    }

    /// Roughly where should the car be in order to shoot at `aim_loc`?
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
    use behavior::RootBehavior;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::bounce_shot::BounceShot;
    use nalgebra::Vector3;

    #[test]
    fn normal() {
        let test = TestRunner::start(
            BounceShot::new(),
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
            BounceShot::new(),
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
}
