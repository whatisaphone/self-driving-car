use behavior::{Action, Behavior, Priority};
use eeg::{color, Drawable};
use rlbot;
use strategy::Context;

pub struct QuickJumpAndDodge {
    start_time: Option<f32>,
    pitch: f32,
    yaw: f32,
    dodge_time: f32,
    phase: Phase,
}

#[derive(Eq, PartialEq)]
enum Phase {
    Jump,
    And,
    Dodge,
    Finished,
}

impl QuickJumpAndDodge {
    const MIN_PHASE_TIME: f32 = 0.05;
    pub const MIN_DODGE_TIME: f32 = Self::MIN_PHASE_TIME * 2.0;

    pub fn new() -> Self {
        Self {
            start_time: None,
            pitch: -1.0,
            yaw: 0.0,
            dodge_time: Self::MIN_DODGE_TIME,
            phase: Phase::Jump,
        }
    }

    pub fn begin(packet: &rlbot::ffi::LiveDataPacket) -> Self {
        Self {
            start_time: Some(packet.GameInfo.TimeSeconds),
            ..Self::new()
        }
    }

    pub fn yaw(mut self, steer: f32) -> Self {
        self.pitch = -1.0;
        self.yaw = steer;
        self
    }

    pub fn angle(mut self, angle: f32) -> Self {
        self.pitch = -angle.cos();
        self.yaw = angle.sin();
        self
    }

    pub fn dodge_time(mut self, dodge_time: f32) -> Self {
        self.dodge_time = dodge_time.max(Self::MIN_DODGE_TIME);
        self
    }
}

impl Behavior for QuickJumpAndDodge {
    fn name(&self) -> &str {
        stringify!(QuickJumpAndDodge)
    }

    fn priority(&self) -> Priority {
        Priority::Force
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let start_time = *self
            .start_time
            .get_or_insert(ctx.packet.GameInfo.TimeSeconds);
        let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;

        ctx.eeg.draw(Drawable::print(
            format!("pitch: {:.0}°", self.pitch.to_degrees()),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("yaw: {:.0}°", self.yaw.to_degrees()),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("dodge_time: {:.2}", self.dodge_time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("elapsed: {:.2}", elapsed),
            color::GREEN,
        ));

        let mut result = rlbot::ffi::PlayerInput::default();

        if self.phase == Phase::Jump || elapsed < self.dodge_time - Self::MIN_PHASE_TIME {
            if self.phase == Phase::Jump && !ctx.me().OnGround {
                ctx.eeg.log("[QuickJumpAndDodge] wheels must be on ground");
                return Action::Abort;
            }

            self.phase = Phase::And;

            result.Jump = true;
            Action::Yield(result)
        } else if self.phase == Phase::And || elapsed < self.dodge_time {
            if ctx.me().DoubleJumped {
                ctx.eeg.log("[QuickJumpAndDodge] must have air charge");
                return Action::Abort;
            }

            self.phase = Phase::Dodge;

            Action::Yield(result)
        } else if self.phase == Phase::Dodge || elapsed < self.dodge_time + Self::MIN_PHASE_TIME {
            if ctx.me().OnGround {
                ctx.eeg.log("[QuickJumpAndDodge] goomba stomped");
                return Action::Abort;
            }

            self.phase = Phase::Finished;

            result.Jump = true;
            result.Pitch = self.pitch;
            result.Yaw = self.yaw;
            Action::Yield(result)
        } else {
            Action::Return
        }
    }
}
