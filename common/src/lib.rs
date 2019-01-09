#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![deny(clippy::all)]

pub use crate::{
    ext::ExtendRLBot,
    polyfill::ExtendDuration,
    pretty::{Angle, AngularVelocity, ControllerInput, Coordinate, Distance, PrettyPrint, Time},
};

pub mod ext;
pub mod kinematics;
pub mod math;
pub mod physics;
mod polyfill;
pub mod prelude;
mod pretty;
pub mod rl;
pub mod rotation;
