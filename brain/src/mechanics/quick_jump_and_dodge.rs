use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct QuickJumpAndDodge {
    start_time: f32,
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

    pub fn begin(packet: &rlbot::LiveDataPacket) -> Self {
        Self {
            start_time: packet.GameInfo.TimeSeconds,
            pitch: -1.0,
            yaw: 0.0,
            dodge_time: Self::MIN_DODGE_TIME,
            phase: Phase::Jump,
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

    pub fn dodge_time(mut self, mut dodge_time: f32) -> Self {
        if dodge_time < Self::MIN_DODGE_TIME {
            warn!("dodge_time too low");
            dodge_time = Self::MIN_DODGE_TIME;
        }
        self.dodge_time = dodge_time;
        self
    }
}

impl Behavior for QuickJumpAndDodge {
    fn name(&self) -> &'static str {
        stringify!(QuickJumpAndDodge)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;

        eeg.draw(Drawable::print(
            format!("pitch: {:.0}°", self.pitch.to_degrees()),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("yaw: {:.0}°", self.yaw.to_degrees()),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("dodge_time: {:.2}", self.dodge_time),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("elapsed: {:.2}", elapsed),
            color::GREEN,
        ));

        let mut result = rlbot::PlayerInput {
            Throttle: 1.0,
            ..Default::default()
        };

        if self.phase == Phase::Jump || elapsed < self.dodge_time - Self::MIN_PHASE_TIME {
            self.phase = Phase::And;
            result.Jump = true;
            Action::Yield(result)
        } else if self.phase == Phase::And || elapsed < self.dodge_time {
            self.phase = Phase::Dodge;
            Action::Yield(result)
        } else if self.phase == Phase::Dodge || elapsed < self.dodge_time + Self::MIN_PHASE_TIME {
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
