#![cfg_attr(feature = "strict", deny(warnings))]

extern crate csv;
extern crate flatbuffers;
extern crate nalgebra;
extern crate rlbot;

pub use collector2::Collector;
pub use data::Snapshot;
pub use rlbot_ext::get_packet_and_inject_rigid_body_tick;
pub use utils::ExtendRotation3;

mod collector2;
mod data;
mod rlbot_ext;
mod utils;
