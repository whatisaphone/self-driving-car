use crate::{
    strategy::{Action, Behavior, Context},
    utils::Stopwatch,
};
use common::physics;
use nalgebra::UnitQuaternion;
use nameof::name_of_type;
use rlbot;

pub struct JumpAndTurn {
    jump_duration: f32,
    total_duration: f32,
    target_rot: UnitQuaternion<f32>,
    time: Stopwatch,
}

impl JumpAndTurn {
    const MIN_JUMP_TIME: f32 = 6.0 / 120.0;
    const MIN_RELEASE_TIME: f32 = 6.0 / 120.0;
    pub const MIN_DURATION: f32 = Self::MIN_JUMP_TIME + Self::MIN_RELEASE_TIME;

    pub fn new(jump_duration: f32, total_duration: f32, target_rot: UnitQuaternion<f32>) -> Self {
        let jump_duration = jump_duration.max(Self::MIN_JUMP_TIME);
        let total_duration = total_duration.max(jump_duration + Self::MIN_RELEASE_TIME);
        Self {
            jump_duration,
            total_duration,
            target_rot,
            time: Stopwatch::new(),
        }
    }
}

impl Behavior for JumpAndTurn {
    fn name(&self) -> &str {
        name_of_type!(JumpAndTurn)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        let elapsed = self.time.tick(ctx.packet.GameInfo.TimeSeconds);
        if elapsed >= self.total_duration {
            return Action::Return;
        }

        ctx.eeg.print_value("target_rot", self.target_rot);

        let jump = elapsed < self.jump_duration;
        let (pitch, yaw, roll) = dom::get_pitch_yaw_roll(
            ctx.me(),
            physics::car_forward_axis(self.target_rot),
            physics::car_roof_axis(self.target_rot),
        );

        Action::Yield(rlbot::ffi::PlayerInput {
            Pitch: pitch,
            Yaw: yaw,
            Roll: roll,
            Jump: jump,
            ..Default::default()
        })
    }
}
