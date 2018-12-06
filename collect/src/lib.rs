#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
extern crate csv;
extern crate flatbuffers;
extern crate nalgebra;
extern crate rlbot;

pub use crate::{
    collector2::Collector,
    data2::{RecordingPlayerTick, RecordingRigidBodyState, RecordingTick},
    rlbot_ext::get_packet_and_inject_rigid_body_tick,
};

mod collector2;
mod data2;
mod rlbot_ext;
