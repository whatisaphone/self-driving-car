#![cfg_attr(feature = "strict", deny(warnings))]

extern crate arrayvec;
extern crate chip;
extern crate collect;
extern crate common;
extern crate crossbeam_channel;
#[macro_use]
extern crate derive_new;
#[cfg(test)]
extern crate flatbuffers;
extern crate graphics;
extern crate itertools;
#[macro_use]
extern crate lazy_static;
extern crate lazycell;
#[macro_use]
extern crate log;
extern crate nalgebra;
extern crate ncollide3d;
extern crate ordered_float;
extern crate piston_window;
extern crate rlbot;
extern crate simulate;

pub use brain::Brain;
pub use eeg::EEG;

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
