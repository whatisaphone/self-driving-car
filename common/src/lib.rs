#![cfg_attr(feature = "strict", deny(warnings))]

#[cfg(test)]
extern crate chip;
#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate rlbot;

pub use ext::ExtendRLBot;
pub use pretty::PrettyPrint;

pub mod ext;
pub mod physics;
pub mod prelude;
mod pretty;
pub mod rl;
pub mod rotation;
