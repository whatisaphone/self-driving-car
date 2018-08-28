use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct QuickJumpAndDodge {
    yaw: f32,
    start_time: f32,
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
    pub fn begin(packet: &rlbot::LiveDataPacket) -> QuickJumpAndDodge {
        QuickJumpAndDodge {
            yaw: 0.0,
            start_time: packet.GameInfo.TimeSeconds,
            phase: Phase::Jump,
        }
    }

    pub fn yaw(mut self, steer: f32) -> QuickJumpAndDodge {
        self.yaw = steer;
        self
    }
}

impl Behavior for QuickJumpAndDodge {
    fn name(&self) -> &'static str {
        "QuickJumpAndDodge"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;

        eeg.draw(Drawable::print(self.name(), color::YELLOW));
        eeg.draw(Drawable::print(
            format!("elapsed: {:.2}", elapsed),
            color::GREEN,
        ));

        let mut result = rlbot::PlayerInput {
            Throttle: 1.0,
            ..Default::default()
        };

        if self.phase == Phase::Jump || elapsed < 0.05 {
            self.phase = Phase::And;
            result.Jump = true;
            Action::Yield(result)
        } else if self.phase == Phase::And || elapsed < 0.10 {
            self.phase = Phase::Dodge;
            Action::Yield(result)
        } else if self.phase == Phase::Dodge || elapsed < 0.15 {
            self.phase = Phase::Finished;
            result.Jump = true;
            result.Pitch = -1.0;
            result.Yaw = self.yaw;
            Action::Yield(result)
        } else {
            Action::Return
        }
    }
}
