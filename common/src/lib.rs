#![cfg_attr(feature = "strict", deny(warnings))]

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
