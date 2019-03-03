use crate::{
    behavior::{
        movement::simple_steer_towards,
        taunt::podium_utils::{PodiumQuickChat, PodiumTimeTracker},
    },
    strategy::{Action, Behavior, Context, Priority},
};
use nalgebra::Point2;
use nameof::name_of_type;

pub struct PodiumStare {
    time_tracker: PodiumTimeTracker,
    chat: PodiumQuickChat,
}

impl PodiumStare {
    pub fn new() -> Self {
        Self {
            time_tracker: PodiumTimeTracker::new(),
            chat: PodiumQuickChat::new(),
        }
    }
}

impl Behavior for PodiumStare {
    fn name(&self) -> &str {
        name_of_type!(PodiumStare)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let elapsed = self.time_tracker.update(ctx);

        self.chat.run(ctx);

        if elapsed < 0.1 {
            return Action::Yield(Default::default());
        }

        let target = if elapsed < 1.5 {
            Point2::new(4000.0, 1600.0)
        } else if elapsed < 3.0 {
            Point2::new(4000.0, -400.0)
        } else {
            Point2::new(4000.0, 800.0)
        };
        let steer = simple_steer_towards(&ctx.me().Physics, target);
        if steer.abs() >= 3.0_f32.to_radians() {
            Action::Yield(common::halfway_house::PlayerInput {
                Steer: steer * 2.0,
                Boost: true,
                ..Default::default()
            })
        } else {
            Action::Yield(Default::default())
        }
    }
}

#[cfg(test)]
mod demo {
    use crate::{
        behavior::taunt::podium_stare::PodiumStare,
        integration_tests::{TestRunner, TestScenario},
    };
    use nalgebra::Point3;

    #[test]
    #[ignore(note = "not a test; just a demo")]
    fn demo_podium_stare() {
        TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1000.0, 0.0, 92.0),
                car_loc: Point3::new(0.0, 0.0, 0.0),
                ..Default::default()
            })
            .behavior(PodiumStare::new())
            .run_for_millis(5000);
    }
}
