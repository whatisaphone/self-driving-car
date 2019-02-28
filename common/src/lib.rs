#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]

pub use crate::{
    ext::ExtendRLBot,
    flatbuffers::vector_iter,
    polyfill::ExtendDuration,
    pretty::{
        Angle, AngularVelocity, ControllerInput, Coordinate, Distance, PrettyPrint, Speed, Time,
    },
};

pub mod ext;
mod flatbuffers;
pub mod halfway_house;
pub mod kinematics;
pub mod math;
pub mod physics;
mod polyfill;
pub mod prelude;
mod pretty;
pub mod rl;
pub mod rotation;
