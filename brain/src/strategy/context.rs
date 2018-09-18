use behavior::Behavior;
use rlbot;
use std::collections::VecDeque;
use strategy::Runner2;
use utils::{my_car, one_v_one};
use EEG;

pub struct Context<'a> {
    pub packet: &'a rlbot::LiveDataPacket,
    pub eeg: &'a mut EEG,
}

impl<'a> Context<'a> {
    pub fn new(packet: &'a rlbot::LiveDataPacket, eeg: &'a mut EEG) -> Self {
        Self { packet, eeg }
    }

    /// Return the player we are controlling.
    pub fn me(&self) -> &'a rlbot::PlayerInfo {
        my_car(self.packet)
    }

    /// Assert that the game is a 1v1, and return a tuple of (me, enemy).
    pub fn one_v_one(&self) -> (&'a rlbot::PlayerInfo, &'a rlbot::PlayerInfo) {
        if self.packet.NumCars != 2 {
            panic!("expected a 1v1, but found {} players", self.packet.NumCars);
        }

        one_v_one(self.packet)
    }
}
