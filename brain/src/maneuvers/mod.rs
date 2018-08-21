pub use self::bounce_shot::BounceShot;
use eeg::EEG;
pub use maneuvers::fifty_fifty::FiftyFifty;
use rlbot;

mod bounce_shot;
mod fifty_fifty;

pub trait Maneuver {
    fn execute(&self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput;
}
