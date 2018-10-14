use behavior::{Action, Behavior};
use common::ext::ExtendPhysics;
use eeg::{color, Drawable};
use mechanics::{simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use nalgebra::Vector2;
use predict::intercept::estimate_intercept_car_ball;
use rules::SameBallTrajectory;
use std::f32::consts::PI;
use strategy::Context;
use utils::{my_goal_center, ExtendF32, ExtendVector2, ExtendVector3};

pub struct FiftyFifty {
    same_ball_trajectory: SameBallTrajectory,
}

impl FiftyFifty {
    const BALL_MAX_Z: f32 = 130.0;

    pub fn new() -> FiftyFifty {
        FiftyFifty {
            same_ball_trajectory: SameBallTrajectory::new(),
        }
    }
}

impl Behavior for FiftyFifty {
    fn name(&self) -> &str {
        stringify!(FiftyFifty)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        return_some!(self.same_ball_trajectory.execute(ctx));

        let me = ctx.me();

        let intercept =
            estimate_intercept_car_ball(ctx, me, |_t, loc, _vel| loc.z < Self::BALL_MAX_Z);

        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[FiftyFifty] unknown intercept");
            return Action::Abort;
        });

        let target_angle = blocking_angle(
            intercept.ball_loc.to_2d(),
            me.Physics.loc().to_2d(),
            my_goal_center(),
            PI / 6.0,
        );
        let target_loc = intercept.ball_loc.to_2d() + Vector2::unit(target_angle) * 200.0;
        let target_dist = (target_loc - me.Physics.loc().to_2d()).norm();

        ctx.eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        ctx.eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));

        if intercept.time >= 0.2 {
            // TODO: this is not how this works…
            let mut child =
                GroundAccelToLoc::new(target_loc, ctx.packet.GameInfo.TimeSeconds + intercept.time);
            child.execute2(ctx)
        } else {
            let angle = simple_yaw_diff(&me.Physics, intercept.ball_loc.to_2d());
            Action::call(QuickJumpAndDodge::begin(ctx.packet).angle(angle))
        }
    }
}

/// Calculate an angle from `car_loc` to `ball_loc`, trying to get between
/// `ball_loc` and `block_loc`, but not adjusting the approach angle by more
/// than `max_angle_diff`.
fn blocking_angle(
    ball_loc: Vector2<f32>,
    car_loc: Vector2<f32>,
    block_loc: Vector2<f32>,
    max_angle_diff: f32,
) -> f32 {
    let naive_angle = ball_loc.angle_to(car_loc);
    let block_angle = ball_loc.angle_to(block_loc);
    let adjust = (block_angle - naive_angle)
        .normalize_angle()
        .max(-max_angle_diff)
        .min(max_angle_diff);
    (naive_angle + adjust).normalize_angle()
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::FiftyFifty;
    use nalgebra::Vector3;

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            FiftyFifty::new(),
            TestScenario {
                car_loc: Vector3::new(256.0, -3839.98, 17.01),
                ..Default::default()
            },
        );

        test.sleep_millis(5500);

        assert!(test.has_scored());
    }
}
