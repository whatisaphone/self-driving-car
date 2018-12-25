pub use self::{
    defense::{defensive_hit, Defense, PushToOwnCorner},
    panic_defense::PanicDefense,
    retreat::Retreat,
};

#[allow(clippy::module_inception)]
mod defense;
mod panic_defense;
mod retreat;
mod retreating_save;
