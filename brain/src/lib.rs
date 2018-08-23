extern crate bakkesmod;
extern crate crossbeam_channel;
extern crate graphics;
#[macro_use]
extern crate lazy_static;
extern crate collect;
extern crate nalgebra;
extern crate piston_window;
extern crate rlbot;
extern crate simulate;

pub use brain::Brain;

mod behavior;
mod behaviors;
mod brain;
mod eeg;
#[cfg(test)]
mod integration_tests;
mod maneuvers;
mod mechanics;
mod predict;
mod utils;
