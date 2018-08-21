extern crate bakkesmod;
extern crate crossbeam_channel;
extern crate graphics;
extern crate nalgebra;
extern crate piston_window;
extern crate rlbot;
extern crate simulate;
#[macro_use]
extern crate lazy_static;

pub use brain::Brain;

mod brain;
mod eeg;
mod maneuvers;
mod mechanics;
mod predict;
#[cfg(test)]
mod tests;
mod utils;
