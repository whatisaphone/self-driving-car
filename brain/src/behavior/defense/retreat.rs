use crate::{
    behavior::{
        defense::{PanicDefense, PushToOwnCorner},
        higher_order::TryChoose,
    },
    strategy::{Action, Behavior, Context, Priority, Scenario},
};
use common::prelude::*;
use nameof::name_of_type;

pub struct Retreat;

impl Retreat {
    pub fn new() -> Self {
        Retreat
    }

    /// Returns `true` if the ball is between me and my goal.
    pub fn behind_ball(ctx: &mut Context<'_>) -> bool {
        let intercept = some_or_else!(ctx.scenario.me_intercept(), {
            return false;
        });
        let ball_loc = intercept.ball_loc.to_2d();
        let goal_loc = ctx.game.own_goal().center_2d;
        let goal_to_ball_axis = (ball_loc - goal_loc).to_axis();

        let ball_dist = (ball_loc - goal_loc).norm();
        let me_dist = (ctx.me().Physics.loc_2d() - goal_loc).dot(&goal_to_ball_axis);
        me_dist > ball_dist + 500.0
    }
}

impl Behavior for Retreat {
    fn name(&self) -> &str {
        name_of_type!(Retreat)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let mut choices = Vec::<Box<dyn Behavior>>::new();

        if Self::behind_ball(ctx) && ctx.scenario.possession() >= -Scenario::POSSESSION_CONTESTABLE
        {
            choices.push(Box::new(PushToOwnCorner::new()));
        }

        choices.push(Box::new(PanicDefense::new()));

        Action::tail_call(TryChoose::new(Priority::Idle, choices))
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::helpers::TestRunner;
    use brain_test_data::recordings;

    #[test]
    fn retreating_hit_to_own_corner() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::RETREATING_HIT_TO_OWN_CORNER, 104.5)
            .starting_boost(74.0)
            .soccar()
            .run_for_millis(4500);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.Location.X < -1500.0);
        assert!(packet.GameBall.Physics.Velocity.X < -500.0);
        assert!(!test.enemy_has_scored());
    }
}
