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
        game: &'a Game<'_>,
        ball_predictor: &'a dyn BallPredictor,
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

    /// I should not have mixed immumtable and mutable values in the `Context`.
    /// This is part of the pathway towards fixing that mistake.
    pub fn split<'s>(&'s mut self) -> (Context2<'a, 's>, &'s mut EEG) {
        let ctx = Context2 {
            packet: self.packet,
            game: self.game,
            scenario: &self.scenario,
        };
        (ctx, self.eeg)
    }
}

pub struct Context2<'c, 's> {
    pub packet: &'c rlbot::ffi::LiveDataPacket,
    pub game: &'c Game<'c>,
    pub scenario: &'s Scenario<'c>,
}

impl<'c, 's> Context2<'c, 's> {
    /// Return the player we are controlling.
    pub fn me(&self) -> &rlbot::ffi::PlayerInfo {
        self.game.me()
    }
}
