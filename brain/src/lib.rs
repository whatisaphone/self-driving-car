extern crate bakkesmod;
extern crate collect;
extern crate crossbeam_channel;
#[cfg(test)]
extern crate csv;
extern crate graphics;
#[cfg(test)]
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate log;
extern crate nalgebra;
extern crate ncollide2d;
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

mod behavior;
mod brain;
mod eeg;
#[cfg(test)]
mod integration_tests;
mod maneuvers;
mod mechanics;
mod predict;
mod rules;
mod utils;
