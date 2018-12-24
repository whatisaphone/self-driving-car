use crate::{
    behavior::{Action, Behavior},
    routing::{
        behavior::FollowRoute,
        plan::{ChainedPlanner, GroundDrive, TurnPlanner},
    },
    strategy::Context,
};
use common::prelude::*;
use nalgebra::{Point2, Vector2};
use nameof::name_of_type;

pub struct ResetBehindBall {
    loc: Point2<f32>,
    distance: f32,
    never_recover: bool,
}

impl ResetBehindBall {
    pub fn behind_loc(loc: Point2<f32>) -> Self {
        Self {
            loc,
            distance: 1500.0,
            never_recover: false,
        }
    }

    pub fn distance(mut self, distance: f32) -> Self {
        self.distance = distance;
        self
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

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let target_loc = self.get_sane_drive_loc(ctx);
        let straight = GroundDrive::new(target_loc).end_chop(0.5);
        let turn = TurnPlanner::new(self.loc, None);
        let chain = ChainedPlanner::chain(vec![Box::new(straight), Box::new(turn)]);
        Action::call(FollowRoute::new(chain).never_recover(self.never_recover))
    }
}

impl ResetBehindBall {
    fn get_sane_drive_loc(&self, ctx: &mut Context) -> Point2<f32> {
        let mut target_loc =
            self.loc + (self.loc - ctx.game.enemy_goal().center_2d).normalize() * self.distance;

        if !ctx.game.is_inside_field(target_loc) {
            ctx.eeg
                .log("[ResetBehindBall] loc outside field; trying straight back from reference");
            target_loc = self.loc
                + Vector2::new(
                    0.0,
                    ctx.game.own_goal().center_2d.y.signum() * self.distance,
                );
        }

        if !ctx.game.is_inside_field(target_loc) {
            ctx.eeg
                .log("[ResetBehindBall] loc outside field; trying straight back from self");
            target_loc = ctx.me().Physics.loc_2d()
                + Vector2::new(
                    0.0,
                    ctx.game.own_goal().center_2d.y.signum() * self.distance,
                );
        }

        if !ctx.game.is_inside_field(target_loc) {
            ctx.eeg
                .log("[ResetBehindBall] loc outside field; going to goal instead");
            return ctx.game.own_goal().center_2d
                + Vector2::new(0.0, ctx.game.own_goal().center_2d.y.signum() * 250.0);
        }

        let margin = 250.0;
        let max_x = ctx.game.field_max_x() - margin;
        if target_loc.x.abs() >= max_x {
            ctx.eeg.log("[ResetBehindBall] clamping x");
            target_loc.x = max_x * target_loc.x.signum();
        }
        let max_y = ctx.game.field_max_y() - margin;
        if target_loc.y.abs() >= max_y {
            ctx.eeg.log("[ResetBehindBall] clamping y");
            target_loc.y = max_y * target_loc.y.signum();
        }
        target_loc
    }
}
