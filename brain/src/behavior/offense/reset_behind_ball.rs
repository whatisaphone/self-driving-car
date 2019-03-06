use crate::{
    routing::{
        behavior::FollowRoute,
        plan::{GetDollar, GroundDrive},
    },
    strategy::{Action, Behavior, BoostPickup, Context},
};
use common::prelude::*;
use nalgebra::{Point2, Vector2};
use nameof::name_of_type;
use simulate::linear_interpolate;

pub struct ResetBehindBall {
    loc: Point2<f32>,
    distance: f32,
    never_recover: bool,
}

impl ResetBehindBall {
    pub fn behind_loc(loc: Point2<f32>, distance: f32) -> Self {
        Self {
            loc,
            distance,
            never_recover: false,
        }
    }

    pub fn never_recover(mut self, never_recover: bool) -> Self {
        self.never_recover = never_recover;
        self
    }
}

impl Behavior for ResetBehindBall {
    fn name(&self) -> &str {
        name_of_type!(ResetBehindBall)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let target_loc = self.get_sane_drive_loc(ctx);
        if let Some(pickup) = self.snap_to_boost_if_close(ctx, target_loc) {
            return Action::tail_call(FollowRoute::new(GetDollar::new(self.loc).pickup(pickup)));
        }

        let straight = GroundDrive::new(target_loc)
            .end_chop(0.5)
            .always_prefer_dodge(true);
        Action::tail_call(
            FollowRoute::new(straight)
                .same_ball_trajectory(true)
                .never_recover(self.never_recover),
        )
    }
}

impl ResetBehindBall {
    fn get_sane_drive_loc(&self, ctx: &mut Context<'_>) -> Point2<f32> {
        let ball_vel = ctx.packet.GameBall.Physics.vel_2d();
        let ball_vel_towards_my_half = ball_vel.dot(&-ctx.game.own_goal().normal_2d);
        let offensive_stance = ball_vel_towards_my_half < 1000.0;
        let direction = if offensive_stance {
            ctx.eeg
                .log(name_of_type!(ResetBehindBall), "offensive stance");
            self.loc - ctx.game.enemy_goal().center_2d
        } else {
            ctx.eeg
                .log(name_of_type!(ResetBehindBall), "defensive stance");
            ctx.game.own_goal().center_2d - self.loc
        };
        let mut target_loc = self.loc + direction.normalize() * self.distance;

        let margin = 1000.0;
        let max_x = ctx.game.field_max_x() - margin;
        let max_y = ctx.game.field_max_y() - margin;

        if !ctx.game.is_inside_field(target_loc) {
            ctx.eeg.log(
                self.name(),
                "loc outside field; trying straight back from reference",
            );
            target_loc = Point2::new(
                target_loc.x.max(-max_x).min(max_x),
                target_loc.y + ctx.game.own_goal().center_2d.y.signum() * self.distance,
            );
        }

        if !ctx.game.is_inside_field(target_loc) {
            ctx.eeg.log(
                self.name(),
                "loc outside field; trying straight back from self",
            );
            target_loc = ctx.me().Physics.loc_2d()
                + Vector2::new(
                    0.0,
                    ctx.game.own_goal().center_2d.y.signum() * self.distance,
                );
        }

        if !ctx.game.is_inside_field(target_loc) {
            ctx.eeg
                .log(self.name(), "loc outside field; going to goal instead");
            return ctx.game.own_goal().center_2d
                + Vector2::new(0.0, ctx.game.own_goal().center_2d.y.signum() * 250.0);
        }

        if target_loc.x.abs() >= max_x {
            ctx.eeg.log(self.name(), "clamping x");
            target_loc.x = max_x * target_loc.x.signum();
        }
        if target_loc.y.abs() >= max_y {
            ctx.eeg.log(self.name(), "clamping y");
            target_loc.y = max_y * target_loc.y.signum();
        }
        target_loc
    }

    fn snap_to_boost_if_close<'ctx>(
        &self,
        ctx: &mut Context<'ctx>,
        loc: Point2<f32>,
    ) -> Option<&'ctx BoostPickup> {
        if ctx.me().Boost >= 50 {
            return None;
        }
        let distance = linear_interpolate(&[0.0, 50.0], &[1000.0, 250.0], ctx.me().Boost as f32);
        for pickup in ctx.game.boost_dollars() {
            if (loc - pickup.loc).norm() < distance
                && (pickup.loc - ctx.me().Physics.loc_2d()).norm()
                    >= (loc - ctx.me().Physics.loc_2d()).norm()
            {
                ctx.eeg
                    .log(name_of_type!(ResetBehindBall), "snapping to boost");
                return Some(pickup);
            }
        }
        return None;
    }
}
