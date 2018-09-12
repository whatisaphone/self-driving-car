use behavior::{Action, Behavior};
use eeg::EEG;
use rlbot;

/// Run a [`Behavior`] until it returns, then do nothing forever.
pub struct Fuse {
    child: Option<Box<Behavior>>,
}

impl Fuse {
    pub fn new(child: Box<Behavior>) -> Fuse {
        Fuse { child: Some(child) }
    }
}

impl Behavior for Fuse {
    fn name(&self) -> &'static str {
        stringify!(Fuse)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        // `take()` leaves a None behind, so this can only match `Some` once.
        match self.child.take() {
            Some(b) => Action::Call(b),
            None => Action::Yield(Default::default()),
        }
    }
}

pub struct Delay {
    time: f32,
    child: Option<Box<Behavior>>,
}

impl Delay {
    pub fn new(time: f32, child: Box<Behavior>) -> Delay {
        Delay {
            time,
            child: Some(child),
        }
    }
}

impl Behavior for Delay {
    fn name(&self) -> &'static str {
        stringify!(Delay)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if packet.GameInfo.TimeSeconds < self.time {
            Action::Yield(Default::default())
        } else {
            // `take()` leaves a None behind, so this can only match `Some` once.
            match self.child.take() {
                Some(b) => Action::Call(b),
                None => Action::Yield(Default::default()),
            }
        }
    }
}
