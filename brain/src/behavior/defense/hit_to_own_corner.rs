use crate::{
    behavior::{
        higher_order::Chain,
        strike::{GroundedHit, GroundedHitAimContext, GroundedHitTarget, GroundedHitTargetAdjust},
    },
    eeg::Event,
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
        HitToOwnCorner
    }
}

impl Behavior for HitToOwnCorner {
    fn name(&self) -> &str {
        name_of_type!(HitToOwnCorner)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg.track(Event::HitToOwnCorner);

        Action::call(Chain::new(Priority::Striking, vec![
            Box::new(FollowRoute::new(GroundIntercept::new())),
            Box::new(GroundedHit::hit_towards(Self::aim)),
        ]))
    }
}

impl HitToOwnCorner {
    fn aim(ctx: &mut GroundedHitAimContext) -> Result<GroundedHitTarget, ()> {
        let avoid = ctx.game.own_goal().center_2d;

        let me_loc = ctx.car.Physics.loc_2d();
        let ball_loc = ctx.intercept_ball_loc.to_2d();
        let me_to_ball = ball_loc - me_loc;

        let ltr_dir = Rotation2::new(PI / 6.0) * me_to_ball;
        let ltr = WallRayCalculator::calculate(ball_loc, ball_loc + ltr_dir);
        let rtl_dir = Rotation2::new(-PI / 6.0) * me_to_ball;
        let rtl = WallRayCalculator::calculate(ball_loc, ball_loc + rtl_dir);

        let result = if (avoid - ltr).norm() > (avoid - rtl).norm() {
            ctx.eeg.log("push from left to right");
            ltr
        } else {
            ctx.eeg.log("push from right to left");
            rtl
        };

        match WallRayCalculator::wall_for_point(ctx.game, result) {
            Wall::OwnGoal => {
                ctx.eeg.log("avoiding the own goal");
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
