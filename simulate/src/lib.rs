#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate ncollide3d;
extern crate rlbot;

pub use car::{Car, CarSimulateError};
pub use car1d::Car1D;
pub use car_aerial_60deg::CarAerial60Deg;
pub use car_powerslide_turn::{CarPowerslideTurn, CarPowerslideTurnPlan};
pub use collision::ball_car_distance;
pub use common::rl;
pub use math::linear_interpolate;

macro_rules! some_or_else {
    ($e:expr, $b:block) => {
        match $e {
            Some(x) => x,
            None => $b,
        };
    };
}

mod car;
mod car1d;
mod car_aerial_60deg;
mod car_powerslide_turn;
pub mod car_single_jump;
mod collision;
mod math;
mod tables;
