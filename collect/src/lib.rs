#![cfg_attr(feature = "strict", deny(warnings))]

pub use crate::{
    collector::Collector,
    data::{RecordingPlayerTick, RecordingRigidBodyState, RecordingTick},
    rlbot_ext::get_packet_and_inject_rigid_body_tick,
};

mod collector;
mod data;
mod rlbot_ext;
