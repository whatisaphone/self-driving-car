pub use routing::segments::{null::NullSegment, simple_arc::SimpleArc, straight::Straight};

mod null;
#[allow(dead_code)] // Pending inclusion. Put me in coach!
mod powerslide_turn;
mod simple_arc;
mod straight;
