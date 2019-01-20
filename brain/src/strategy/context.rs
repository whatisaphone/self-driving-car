use crate::{
    eeg::EEG,
    strategy::{game::Game, scenario::Scenario, Team},
};
use common::prelude::*;

pub struct Context<'a> {
    pub packet: &'a rlbot::ffi::LiveDataPacket,
    pub game: &'a Game<'a>,
    pub scenario: &'a Scenario<'a>,
    pub eeg: &'a mut EEG,
}

impl<'a> Context<'a> {
    pub fn new(
        game: &'a Game<'_>,
        packet: &'a rlbot::ffi::LiveDataPacket,
        scenario: &'a Scenario<'a>,
        eeg: &'a mut EEG,
    ) -> Self {
        Self {
            packet,
            game,
            scenario,
            eeg,
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

    pub fn quick_chat(&mut self, probability: f32, choices: &[rlbot::flat::QuickChatSelection]) {
        // Use physics data to keep things deterministic :)
        let random1 = self.me().Physics.loc_2d().x.abs().fract();
        let random2 = self.me().Physics.loc_2d().y.abs().fract();
        if random1 < probability {
            let choice = choices[(random2 * choices.len() as f32) as usize];
            self.eeg.quick_chat(choice);
        }
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
