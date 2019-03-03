use crate::{
    behavior::{
        movement::{simple_steer_towards, Yielder},
        taunt::podium_utils::{PodiumQuickChat, PodiumTimeTracker},
    },
    eeg::Drawable,
    strategy::{Action, Behavior, Context, Priority},
};
use common::prelude::*;
use dom::get_pitch_yaw_roll;
use nalgebra::{Point2, Vector2, Vector3};
use nameof::name_of_type;

pub struct PodiumSpew {
    time_tracker: PodiumTimeTracker,
    chat: PodiumQuickChat,
    child: Option<Yielder>,
}

impl PodiumSpew {
    pub fn new() -> Self {
        Self {
            time_tracker: PodiumTimeTracker::new(),
            chat: PodiumQuickChat::new(),
            child: None,
        }
    }
}

impl Behavior for PodiumSpew {
    fn name(&self) -> &str {
        name_of_type!(PodiumSpew)
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

        let me = ctx.me();
        let center_loc = Point2::new(-4000.0, -600.0);
        let spread = Vector2::new(0.0, 1600.0);

        if elapsed < 0.1 {
            return Action::Yield(Default::default());
        } else if elapsed < 0.5 && me.OnGround {
            self.child = Some(Yielder::new(0.1, common::halfway_house::PlayerInput {
                Jump: true,
                Yaw: 1.0,
                ..Default::default()
            }));
            return self.execute_old(ctx); // Recurse into `run_child`.
        }

        if !me.OnGround {
            let target_loc = center_loc;

            ctx.eeg
                .draw(Drawable::ghost_car_ground(center_loc, me.Physics.rot()));

            let forward = (target_loc - ctx.me().Physics.loc_2d())
                .to_3d(0.0)
                .to_axis();
            let (_pitch, yaw, _roll) = get_pitch_yaw_roll(me, forward, Vector3::z_axis());
            return Action::Yield(common::halfway_house::PlayerInput {
                Yaw: yaw,
                ..Default::default()
            });
        }

        let n = triangle(elapsed * 1.5);
        let n = (n - 0.5) * 2.0;
        let target_loc = center_loc + spread * n;

        ctx.eeg
            .draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));

        let steer = simple_steer_towards(&me.Physics, target_loc);
        Action::Yield(common::halfway_house::PlayerInput {
            Steer: steer * 2.0,
            Boost: true,
            ..Default::default()
        })
    }
}

impl PodiumSpew {
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
}

/// Generates a triangle wave, with period 2 and amplitude 0 to 1.
fn triangle(x: f32) -> f32 {
    1.0 - (x % 2.0 - 1.0).abs()
}

#[cfg(test)]
mod demo {
    use crate::{
        behavior::taunt::podium_spew::PodiumSpew,
        integration_tests::{TestRunner, TestScenario},
    };
    use nalgebra::Point3;

    #[test]
    #[ignore(note = "not a test; just a demo")]
    fn demo_podium_spew() {
        TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1000.0, 0.0, 92.0),
                car_loc: Point3::new(0.0, 0.0, 0.0),
                ..Default::default()
            })
            .behavior(PodiumSpew::new())
            .run_for_millis(5000);
    }
}
