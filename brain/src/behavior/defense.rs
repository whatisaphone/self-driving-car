use behavior::shoot::Shoot;
use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct Defense {
    min_distance: Option<f32>,
}

impl Defense {
    pub fn new() -> Defense {
        Defense { min_distance: None }
    }
}

impl Behavior for Defense {
    fn name(&self) -> &'static str {
        "Defense"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print(self.name(), color::YELLOW));

        // TODO: this is not how this works…
        let mut child = Shoot::new();
        child.execute(packet, eeg)
    }
}
