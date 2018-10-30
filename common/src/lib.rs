#![cfg_attr(feature = "strict", deny(warnings))]

#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate rlbot;

pub mod ext;
pub mod physics;
pub mod prelude;
pub mod rl;
