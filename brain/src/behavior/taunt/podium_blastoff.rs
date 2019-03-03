use crate::{
    behavior::{
        higher_order::Chain,
        movement::Yielder,
        taunt::{
            podium_utils::{PodiumQuickChat, PodiumTimeTracker},
            twirl::Twirl,
        },
    },
    strategy::{Action, Behavior, Context, Priority},
};
use nameof::name_of_type;
use vec_box::vec_box;

pub struct PodiumBlastoff {
    time_tracker: PodiumTimeTracker,
    chat: PodiumQuickChat,
    child: Option<Chain>,
}

impl PodiumBlastoff {
    pub fn new() -> Self {
        Self {
            time_tracker: PodiumTimeTracker::new(),
            chat: PodiumQuickChat::new(),
            child: None,
        }
    }
}

impl Behavior for PodiumBlastoff {
    fn name(&self) -> &str {
        name_of_type!(PodiumBlastoff)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let elapsed = self.time_tracker.update(ctx);

        self.chat.run(ctx);

        if let Some(action) = self.run_child(ctx) {
            return action;
        }

        if elapsed < 0.1 {
            return Action::Yield(Default::default());
        }

        if ctx.me().OnGround {
            return self.jump(ctx);
        }

        let blastoff = elapsed >= 3.75;
        Twirl::new(blastoff).execute_old(ctx)
    }
}

impl PodiumBlastoff {
    fn run_child(&mut self, ctx: &mut Context<'_>) -> Option<Action> {
        if let Some(child) = self.child.as_mut() {
            match child.execute_old(ctx) {
                Action::Yield(i) => {
                    return Some(Action::Yield(i));
                }
                Action::Return => {
                    self.child = None;
                    return None;
                }
                _ => panic!("unsupported child action"),
            }
        }
        None
    }

    fn jump(&mut self, ctx: &mut Context<'_>) -> Action {
        self.child = Some(Chain::new(Priority::Taunt, vec_box![
            Yielder::new(0.1, Default::default()),
            Yielder::new(0.1, common::halfway_house::PlayerInput {
                Pitch: 1.0,
                Jump: true,
                ..Default::default()
            }),
            Yielder::new(0.2, common::halfway_house::PlayerInput {
                Pitch: 1.0,
                ..Default::default()
            })
        ]));
        self.child.as_mut().unwrap().execute_old(ctx)
    }
}

#[cfg(test)]
mod demo {
    use crate::{
        behavior::taunt::podium_blastoff::PodiumBlastoff,
        integration_tests::{TestRunner, TestScenario},
    };
    use nalgebra::Point3;

    #[test]
    #[ignore(note = "not a test; just a demo")]
    fn demo_podium_blastoff() {
        TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1000.0, 0.0, 92.0),
                car_loc: Point3::new(0.0, 0.0, 0.0),
                ..Default::default()
            })
            .behavior(PodiumBlastoff::new())
            .run_for_millis(5000);
    }
}
