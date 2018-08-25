use behavior::behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct NullBehavior;

impl Behavior for NullBehavior {
    fn execute(&mut self, _packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("NullBehavior", color::YELLOW));
        Action::Yield(rlbot::PlayerInput::default())
    }
}
