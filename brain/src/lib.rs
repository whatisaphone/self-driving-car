#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]
#![allow(clippy::unreadable_literal)]

pub use crate::{brain::Brain, eeg::EEG};

macro_rules! return_some {
    ($rule:expr) => {
        if let Some(x) = $rule {
            return x;
        }
    };
}

macro_rules! some_or_else {
    ($e:expr, $b:block) => {
        match $e {
            Some(x) => x,
            None => $b,
        };
    };
}

mod behavior;
mod brain;
mod eeg;
#[cfg(test)]
mod integration_tests;
mod plan;
mod predict;
mod routing;
mod rules;
mod sim;
mod strategy;
mod utils;
