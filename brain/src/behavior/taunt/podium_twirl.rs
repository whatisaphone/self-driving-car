use crate::{
    behavior::movement::Yielder,
    strategy::{Action, Behavior, Context, Priority},
};
use common::{kinematics::kinematic_time, prelude::*, rl};
use dom::get_pitch_yaw_roll;
use nalgebra::Vector3;
use nameof::name_of_type;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

#[derive(Clone)]
pub struct PodiumTwirl {
    start_time: Option<f32>,
    // Keep these in `Arc`s so when we clone ourselves, our clones will share the same state and we
    // won't quickchat multiple times.
    gg: Arc<AtomicBool>,
    nice_moves: Arc<AtomicBool>,
}

impl PodiumTwirl {
    pub fn new() -> Self {
        Self {
            start_time: None,
            gg: Arc::new(AtomicBool::new(false)),
            nice_moves: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl Behavior for PodiumTwirl {
    fn name(&self) -> &str {
        name_of_type!(PodiumTwirl)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let start_time = *self
            .start_time
            .get_or_insert(ctx.packet.GameInfo.TimeSeconds);
        let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;

        if !self.gg.load(Ordering::SeqCst) && elapsed >= 1.3 {
            ctx.quick_chat(1.0, &[rlbot::flat::QuickChatSelection::PostGame_Gg]);
            self.gg.store(true, Ordering::SeqCst);
        }

        if !self.nice_moves.load(Ordering::SeqCst) && elapsed >= 2.0 {
            ctx.quick_chat(1.0, &[rlbot::flat::QuickChatSelection::PostGame_NiceMoves]);
            self.nice_moves.store(true, Ordering::SeqCst);
        }

        let me = ctx.me();

        if me.OnGround {
            return self.jump_then_resume();
        }

        let blastoff = elapsed >= 5.0;
        self.air_twirl(ctx.me(), blastoff)
    }
}

impl PodiumTwirl {
    fn jump_then_resume(&self) -> Action {
        Action::tail_call(chain!(self.priority(), [
            Yielder::new(
                common::halfway_house::PlayerInput {
                    Pitch: 1.0,
                    Jump: true,
                    ..Default::default()
                },
                0.2
            ),
            self.clone(),
        ]))
    }

    // Yeah, none of this actually works because I'm not getting physics data during
    // the podium phase for some reason. I'll leave it around in case that gets
    // fixed one day. At least the quick chat works…
    fn air_twirl(&self, car: &common::halfway_house::PlayerInfo, blastoff: bool) -> Action {
        let me_forward_axis = car.Physics.forward_axis();
        let forward = me_forward_axis
            .to_2d()
            .to_3d()
            .rotation_to(&Vector3::z_axis())
            .powf(2.0 / 3.0)
            * me_forward_axis.to_2d().to_3d();
        let (pitch, _yaw, _roll) = get_pitch_yaw_roll(car, forward, Vector3::z_axis());

        let boost = blastoff || {
            let time_to_ground = kinematic_time(
                -car.Physics.loc().z + rl::OCTANE_NEUTRAL_Z,
                car.Physics.vel().z,
                rl::GRAVITY,
            );
            time_to_ground.unwrap_or(0.0) < 0.5
        };

        Action::Yield(common::halfway_house::PlayerInput {
            Pitch: pitch,
            Yaw: 1.0,
            Roll: -1.0,
            Boost: boost || blastoff,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod demo {
    use crate::{
        behavior::taunt::podium_twirl::PodiumTwirl,
        integration_tests::{TestRunner, TestScenario},
    };
    use nalgebra::Point3;

    #[test]
    #[ignore(note = "not a test; just a demo")]
    fn demo_podium_twirl() {
        TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1000.0, 0.0, 92.0),
                ..Default::default()
            })
            .behavior(PodiumTwirl::new())
            .run_for_millis(4000);
    }
}
