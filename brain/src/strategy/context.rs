use plan::ball::BallPredictor;
use rlbot;
use strategy::{game::Game, scenario::Scenario};
use utils::{my_car, one_v_one};
use EEG;

pub struct Context<'a> {
    pub packet: &'a rlbot::ffi::LiveDataPacket,
    pub game: &'a Game<'a>,
    pub eeg: &'a mut EEG,
    pub scenario: Scenario<'a>,
}

impl<'a> Context<'a> {
    pub fn new(
        game: &'a Game,
        ball_predictor: &'a BallPredictor,
        packet: &'a rlbot::ffi::LiveDataPacket,
        eeg: &'a mut EEG,
    ) -> Self {
        Self {
            packet,
            game,
            eeg,
            scenario: Scenario::new(game, ball_predictor, packet),
        }
    }

    /// Return the player we are controlling.
    pub fn me(&self) -> &'a rlbot::ffi::PlayerInfo {
        my_car(self.packet)
    }

    /// Return the villain.
    pub fn enemy(&self) -> &'a rlbot::ffi::PlayerInfo {
        let (_me, enemy) = self.one_v_one();
        enemy
    }

    /// Assert that the game is a 1v1, and return a tuple of (me, enemy).
    pub fn one_v_one(&self) -> (&'a rlbot::ffi::PlayerInfo, &'a rlbot::ffi::PlayerInfo) {
        if self.packet.NumCars != 2 {
            panic!("expected a 1v1, but found {} players", self.packet.NumCars);
        }

        one_v_one(self.packet)
    }
}
