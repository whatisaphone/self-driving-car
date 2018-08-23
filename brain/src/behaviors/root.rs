use behavior::{Action, Behavior};
use eeg::EEG;
use maneuvers::BounceShot;
use rlbot;

pub struct RootBehavior;

impl RootBehavior {
    pub fn new() -> RootBehavior {
        RootBehavior
    }
}

impl Behavior for RootBehavior {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        Action::call(BounceShot::new())
    }
}
