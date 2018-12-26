pub use self::{
    defense::{defensive_hit, Defense},
    hit_to_own_corner::HitToOwnCorner,
    panic_defense::PanicDefense,
    push_to_own_corner::PushToOwnCorner,
    retreat::Retreat,
};

#[allow(clippy::module_inception)]
mod defense;
mod hit_to_own_corner;
mod panic_defense;
mod push_to_own_corner;
mod retreat;
mod retreating_save;
