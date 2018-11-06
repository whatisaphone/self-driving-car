#![cfg_attr(feature = "strict", deny(warnings))]

#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate rlbot;

pub use pretty::PrettyPrint;

pub mod ext;
pub mod physics;
pub mod prelude;
mod pretty;
pub mod rl;
