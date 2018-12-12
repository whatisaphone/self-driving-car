use crate::{
    behavior::{Action, Behavior},
    routing::{
        behavior::FollowRoute,
        plan::{ChainedPlanner, GroundDrive, TurnPlanner},
    },
    strategy::Context,
};
use nalgebra::Point2;

pub struct ResetBehindBall {
    loc: Point2<f32>,
    never_recover: bool,
}

impl ResetBehindBall {
    pub fn behind_loc(loc: Point2<f32>) -> Self {
        Self {
            loc,
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
        stringify!(ResetBehindBall)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let target_loc = get_sane_drive_loc(ctx, self.loc);
        let straight = GroundDrive::new(target_loc).end_chop(0.5);
        let turn = TurnPlanner::new(self.loc, None);
        let chain = ChainedPlanner::chain(vec![Box::new(straight), Box::new(turn)]);
        Action::call(FollowRoute::new(chain).never_recover(self.never_recover))
    }
}

fn get_sane_drive_loc(ctx: &mut Context, loc: Point2<f32>) -> Point2<f32> {
    let mut target_loc = Point2::new(
        loc.x,
        loc.y + ctx.game.own_goal().center_2d.y.signum() * 1500.0,
    );

    if !ctx.game.is_inside_field(target_loc) {
        ctx.eeg
            .log("[ResetBehindBall] loc outside field; going to goal instead");
        return ctx.game.own_goal().center_2d;
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
