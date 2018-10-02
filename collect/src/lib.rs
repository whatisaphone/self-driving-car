#![cfg_attr(feature = "strict", deny(warnings))]

extern crate csv;
extern crate nalgebra;
extern crate rlbot;

pub use collector::Collector;
pub use data::Snapshot;
pub use utils::ExtendRotation3;

mod collector;
mod data;
mod utils;
