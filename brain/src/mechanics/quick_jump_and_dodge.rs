use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct QuickJumpAndDodge {
    phase: Phase,
    start_time: f32,
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
            phase: Phase::Jump,
            start_time: packet.GameInfo.TimeSeconds,
        }
    }
}

impl Behavior for QuickJumpAndDodge {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("QuickJumpAndDodge", color::YELLOW));

        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;

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
            Action::Yield(result)
        } else {
            Action::Return
        }
    }
}
