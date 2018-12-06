#![cfg_attr(feature = "strict", deny(warnings))]

#[cfg(test)]
extern crate chip;
#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate rlbot;

pub use crate::{ext::ExtendRLBot, polyfill::ExtendDuration, pretty::PrettyPrint};

pub mod ext;
pub mod math;
pub mod physics;
mod polyfill;
pub mod prelude;
mod pretty;
pub mod rl;
pub mod rotation;
