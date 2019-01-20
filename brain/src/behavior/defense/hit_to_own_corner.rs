use crate::{
    behavior::{
        higher_order::Chain,
        movement::{GetToFlatGround, SkidRecover},
        strike::{GroundedHit, GroundedHitAimContext, GroundedHitTarget, GroundedHitTargetAdjust},
    },
    eeg::{color, Drawable, Event},
    routing::{behavior::FollowRoute, plan::GroundIntercept},
    strategy::{Action, Behavior, Context, Priority},
    utils::{Wall, WallRayCalculator},
};
use common::prelude::*;
use nalgebra::Rotation2;
use nameof::name_of_type;
use std::f32::consts::PI;

pub struct HitToOwnCorner;

impl HitToOwnCorner {
    pub const MAX_BALL_Z: f32 = GroundedHitTarget::MAX_BALL_Z;

    pub fn new() -> Self {
        Self
    }
}

impl Behavior for HitToOwnCorner {
    fn name(&self) -> &str {
        name_of_type!(HitToOwnCorner)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg.track(Event::HitToOwnCorner);

        let skid_recover_loc = ctx.scenario.ball_prediction().at_time_or_last(0.1).loc;

        Action::tail_call(Chain::new(Priority::Strike, vec![
            // We do want skid recovery. We don't want ResetBehindBall-type stuff. Just single
            // out the basics for now…
            Box::new(GetToFlatGround::new()),
            Box::new(SkidRecover::new(skid_recover_loc.to_2d())),
            Box::new(FollowRoute::new(GroundIntercept::new()).never_recover(true)),
            Box::new(GroundedHit::hit_towards(Self::aim)),
        ]))
    }
}

impl HitToOwnCorner {
    fn aim(ctx: &mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> {
        let avoid = ctx.game.own_goal().center_2d;

        let me_loc = ctx.car.Physics.loc_2d();
        let ball_loc = ctx.intercept_ball_loc.to_2d();
        let me_to_ball = ball_loc - me_loc;

        let ltr_dir = Rotation2::new(PI / 6.0) * me_to_ball;
        let ltr = WallRayCalculator::calculate(ball_loc, ball_loc + ltr_dir);
        let rtl_dir = Rotation2::new(-PI / 6.0) * me_to_ball;
        let rtl = WallRayCalculator::calculate(ball_loc, ball_loc + rtl_dir);

        let result = if (avoid - ltr).norm() > (avoid - rtl).norm() {
            ctx.eeg.track(Event::PushFromLeftToRight);
            ctx.eeg
                .draw(Drawable::print("push from left to right", color::GREEN));
            ltr
        } else {
            ctx.eeg.track(Event::PushFromRightToLeft);
            ctx.eeg
                .draw(Drawable::print("push from right to left", color::GREEN));
            rtl
        };

        match WallRayCalculator::wall_for_point(ctx.game, result) {
            Wall::OwnGoal => {
                ctx.eeg
                    .log(name_of_type!(HitToOwnCorner), "avoiding the own goal");
                Err(())
            }
            _ => Ok(GroundedHitTarget::new(
                ctx.intercept_time,
                GroundedHitTargetAdjust::RoughAim,
                result,
            )),
        }
    }
}
