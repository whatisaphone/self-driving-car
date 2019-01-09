#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![deny(clippy::all)]

pub use crate::{
    collector::Collector,
    data::{RecordingPlayerTick, RecordingRigidBodyState, RecordingTick},
    rlbot_ext::get_packet_and_inject_rigid_body_tick,
};

mod collector;
mod data;
mod rlbot_ext;
