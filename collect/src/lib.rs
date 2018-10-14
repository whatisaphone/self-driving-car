#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
extern crate csv;
extern crate flatbuffers;
extern crate nalgebra;
extern crate rlbot;

pub use collector2::Collector;
pub use common::ext::ExtendRotation3;
pub use data::Snapshot;
pub use rlbot_ext::get_packet_and_inject_rigid_body_tick;

mod collector2;
mod data;
mod rlbot_ext;
