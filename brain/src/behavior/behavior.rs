use eeg::EEG;
use rlbot;

pub trait Behavior {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action;
}

pub enum Action {
    Yield(rlbot::PlayerInput),
    Call(Box<Behavior>),
    Return,
}

impl Action {
    pub fn call(behavior: impl Behavior + 'static) -> Self {
        Action::Call(Box::new(behavior))
    }
}
