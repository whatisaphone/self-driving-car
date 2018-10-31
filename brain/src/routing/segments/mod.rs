pub use routing::segments::{
    chain::Chain,
    forward_dodge::ForwardDodge,
    simple_arc::SimpleArc,
    straight::{Straight, StraightMode},
    turn::Turn,
};

mod chain;
mod forward_dodge;
#[allow(dead_code)] // Pending inclusion. Put me in coach!
mod powerslide_turn;
mod simple_arc;
mod straight;
mod turn;
