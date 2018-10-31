#![cfg_attr(feature = "strict", deny(warnings))]

extern crate collect;
#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate rlbot;

pub use models::OneVOneScenario;

mod models;
pub mod recordings;
