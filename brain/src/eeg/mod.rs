pub use crate::eeg::eeg::*;

pub mod color;
#[allow(clippy::module_inception)]
mod eeg;
#[allow(dead_code)]
pub mod recipes;
mod window;
