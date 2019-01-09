#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![deny(clippy::all)]
#![allow(clippy::unreadable_literal)]

pub use crate::models::OneVOneScenario;

mod models;
pub mod recordings;
