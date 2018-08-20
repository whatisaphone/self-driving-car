pub use self::bounce_shot::BounceShot;
use eeg::EEG;
use rlbot;

mod bounce_shot;

pub trait Maneuver {
    fn execute(&self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput;
}
