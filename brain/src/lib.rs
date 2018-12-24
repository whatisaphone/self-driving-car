#![cfg_attr(feature = "strict", deny(warnings))]

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
mod maneuvers;
mod mechanics;
mod plan;
mod predict;
mod routing;
mod rules;
mod strategy;
mod utils;
