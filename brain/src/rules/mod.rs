use behavior::Action;
use rlbot;
use EEG;

pub struct Finishable {
    finished: bool,
}

impl Finishable {
    pub fn new() -> Finishable {
        Finishable { finished: false }
    }

    pub fn execute(&mut self, _packet: &rlbot::LiveDataPacket, _eeg: &mut EEG) -> Option<Action> {
        if self.finished {
            return Some(Action::Return);
        }
        None
    }

    pub fn set_finished(&mut self) {
        self.finished = true;
    }
}
