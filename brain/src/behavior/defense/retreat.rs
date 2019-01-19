use crate::{
    behavior::{
        defense::{PanicDefense, PushToOwnCorner},
        higher_order::TryChoose,
    },
    eeg::Event,
    strategy::{Action, Behavior, Context, Priority},
};
use common::prelude::*;
use nameof::name_of_type;

pub struct Retreat;

impl Retreat {
    pub fn new() -> Self {
        Retreat
    }

    /// Returns `true` if the ball is between me and my goal.
    pub fn out_of_position(ctx: &mut Context<'_>) -> bool {
        let intercept = some_or_else!(ctx.scenario.me_intercept(), {
            return false;
        });
        let goal_loc = ctx.game.own_goal().center_2d;
        let ball_loc = intercept.ball_loc.to_2d();
        let me_loc = ctx.me().Physics.loc_2d();

        if ctx.game.own_goal().is_y_within_range(me_loc.y, ..0.0) {
            return true;
        }

        let axis = (me_loc - goal_loc).to_axis();
        let ball_dist = (ball_loc - goal_loc).dot(&axis);
        let me_dist = (me_loc - goal_loc).dot(&axis);
        me_dist > ball_dist + 500.0
    }
}

impl Behavior for Retreat {
    fn name(&self) -> &str {
        name_of_type!(Retreat)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg.track(Event::Retreat);

        let mut choices = Vec::<Box<dyn Behavior>>::new();

        if Self::out_of_position(ctx) {
            choices.push(Box::new(PushToOwnCorner::new()));
        }
        choices.push(Box::new(PanicDefense::new()));

        Action::tail_call(TryChoose::new(Priority::Idle, choices))
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{eeg::Event, integration_tests::helpers::TestRunner};
    use brain_test_data::recordings;
    use common::prelude::*;

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

    #[test]
    fn last_ditch_intercept() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::LAST_DITCH_INTERCEPT, 275.0)
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        test.examine_events(|events| {
            assert!(events.contains(&Event::Retreat));
        });

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x >= 2000.0);
    }
}
