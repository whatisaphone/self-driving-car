pub use self::{
    behavior::{Action, Behavior, Priority},
    kickoff::Kickoff,
};

mod behavior;
pub mod defense;
#[macro_use]
pub mod higher_order;
mod kickoff;
pub mod movement;
pub mod offense;
pub mod runner;
pub mod strike;
