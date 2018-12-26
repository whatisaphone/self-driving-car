use crate::{
    eeg::EEG,
    plan::ball::BallPredictor,
    strategy::{game::Game, scenario::Scenario, Team},
};

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
        self.game.me()
    }

    pub fn cars(&self, team: Team) -> impl Iterator<Item = &rlbot::ffi::PlayerInfo> {
        self.game.cars(team)
    }

    pub fn enemy_cars(&self) -> impl Iterator<Item = &rlbot::ffi::PlayerInfo> {
        self.game.cars(self.game.enemy_team)
    }
}
