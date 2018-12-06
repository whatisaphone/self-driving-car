pub use crate::strategy::{
    context::Context,
    dropshot::Dropshot,
    game::{infer_game_mode, BoostPickup, Game, Goal, Team, Vehicle},
    runner2::Runner2,
    scenario::Scenario,
    soccar::Soccar,
};

mod context;
mod dropshot;
mod game;
#[cfg(test)]
pub mod null;
mod runner2;
mod scenario;
mod soccar;
mod strategy;
