use behavior::{Behavior, BehaviorRunner, RootBehavior};
use eeg::{color, Drawable, EEG};
use nalgebra::clamp;
use rlbot;
use utils::FPSCounter;

pub struct Brain {
    runner: BehaviorRunner,
    fps_counter: FPSCounter,
}

impl Brain {
    pub fn with_root_behavior() -> Self {
        Self::with_behavior(Box::new(RootBehavior::new()))
    }

    pub fn with_behavior(behavior: Box<Behavior>) -> Self {
        Self {
            runner: BehaviorRunner::new(behavior),
            fps_counter: FPSCounter::new(),
        }
    }

    pub fn set_behavior(&mut self, behavior: Box<Behavior>) {
        self.runner = BehaviorRunner::new(behavior);
    }

    pub fn tick(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput {
        self.fps_counter.tick(packet.GameInfo.TimeSeconds);

        eeg.draw(Drawable::print(
            format!("game_time: {:.2}", packet.GameInfo.TimeSeconds),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("fps: {}", format_fps(self.fps_counter.fps())),
            color::GREEN,
        ));

        let mut result = self.runner.execute(&packet, eeg);

        result.Throttle = clamp(result.Throttle, -1.0, 1.0);
        result.Steer = clamp(result.Steer, -1.0, 1.0);
        result.Pitch = clamp(result.Pitch, -1.0, 1.0);
        result.Yaw = clamp(result.Yaw, -1.0, 1.0);
        result.Roll = clamp(result.Roll, -1.0, 1.0);
        result
    }
}

fn format_fps(fps: Option<usize>) -> String {
    fps.map(|x| format!("{:.0}", x))
        .unwrap_or("...".to_string())
}
