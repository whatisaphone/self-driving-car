use eeg::EEG;
use rlbot;

pub trait Behavior {
    fn name(&self) -> &'static str;

    fn capture(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Option<Action> {
        None
    }

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
