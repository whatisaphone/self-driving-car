use behavior::{Action, Behavior};
use eeg::EEG;
use rlbot;

/// Run a [`Behavior`] until it returns, then do nothing forever.
pub struct Once {
    child: Option<Box<Behavior>>,
}

impl Once {
    pub fn new(child: Box<Behavior>) -> Once {
        Once { child: Some(child) }
    }
}

impl Behavior for Once {
    fn name(&self) -> &'static str {
        "Once"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        // `take()` leaves a None behind, so this can only match `Some` once.
        match self.child.take() {
            Some(b) => Action::Call(b),
            None => Action::Yield(Default::default()),
        }
    }
}
