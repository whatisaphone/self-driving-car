use crate::{
    behavior::{
        defense::{retreating_save::RetreatingSave, PanicDefense, PushToOwnCorner},
        higher_order::TryChoose,
        offense::TepidHit,
    },
    eeg::Event,
    strategy::{Action, Behavior, Context, Priority},
};
use common::prelude::*;
use nameof::name_of_type;

pub struct Retreat;

impl Retreat {
    pub fn new() -> Self {
        Self
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
        // PushToOwnCorner might end up in RetreatingSave, so no need to duplicate.
        } else {
            choices.push(Box::new(RetreatingSave::new()));
        }
        choices.push(Box::new(PanicDefense::new()));
        // We should never get this far, but it's here as a fail-safe:
        choices.push(Box::new(TepidHit::new()));

        Action::tail_call(TryChoose::new(Priority::Idle, choices))
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        eeg::Event,
        integration_tests::{TestRunner, TestScenario},
    };
    use brain_test_data::recordings;
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

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

    #[test]
    fn no_infinite_loop() {
        let start_loc = Point3::new(517.51996, -3536.02, 17.01);
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1755.77, 447.0, 93.15),
                ball_vel: Vector3::new(-271.521, -404.061, 0.0),
                car_loc: start_loc,
                car_rot: Rotation3::from_unreal_angles(-0.009636601, 0.49049014, -0.00009285019),
                car_vel: Vector3::new(544.621, 296.531, 8.311),
                enemy_loc: Point3::new(-955.0, 1372.2899, 17.01),
                enemy_rot: Rotation3::from_unreal_angles(-0.009583307, -2.3083153, -0.00014091768),
                enemy_vel: Vector3::new(-932.0109, -1015.0109, 8.301001),
                ..Default::default()
            })
            .starting_boost(80.0)
            .soccar()
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        let car_loc = packet.GameCars[0].Physics.loc();
        let dist_moved = (car_loc.to_2d() - start_loc.to_2d()).norm();
        println!("dist_moved = {:?}", dist_moved);
        assert!(dist_moved >= 1000.0);
    }
}
