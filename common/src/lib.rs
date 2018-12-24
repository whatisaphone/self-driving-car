#![cfg_attr(feature = "strict", deny(warnings))]

pub use crate::{ext::ExtendRLBot, polyfill::ExtendDuration, pretty::PrettyPrint};

pub mod ext;
pub mod math;
pub mod physics;
mod polyfill;
pub mod prelude;
mod pretty;
pub mod rl;
pub mod rotation;
