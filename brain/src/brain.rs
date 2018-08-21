use eeg::{color, Drawable, EEG};
use maneuvers::{BounceShot, Maneuver};
use nalgebra::clamp;
use rlbot;
use utils::fps_counter::FPSCounter;

pub struct Brain {
    maneuver: Box<Maneuver>,
    fps_counter: FPSCounter,
    eeg: EEG,
}

impl Brain {
    pub fn new() -> Self {
        Self::with_maneuver(Box::new(BounceShot))
    }

    pub fn with_maneuver(maneuver: Box<Maneuver>) -> Self {
        Self {
            maneuver,
            fps_counter: FPSCounter::new(),
            eeg: EEG::new(),
        }
    }

    pub fn tick(&mut self, packet: &rlbot::LiveDataPacket) -> rlbot::PlayerInput {
        self.fps_counter.tick(packet.GameInfo.TimeSeconds);

        self.eeg.draw(Drawable::print(
            format!("game_time: {:.1}", packet.GameInfo.TimeSeconds),
            color::GREEN,
        ));
        self.eeg.draw(Drawable::print(
            format!("fps: {}", format_fps(self.fps_counter.fps())),
            color::GREEN,
        ));

        let mut result = self.maneuver.execute(&packet, &mut self.eeg);

        self.eeg.show(packet);

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
