#![cfg_attr(feature = "strict", deny(warnings))]

pub use crate::{
    car::{Car, CarSimulateError},
    car1d::Car1D,
    car_forward_dodge::{CarForwardDodge, CarForwardDodge1D},
    car_powerslide_turn::{CarPowerslideTurn, CarPowerslideTurnBlueprint},
    collision::ball_car_distance,
    math::linear_interpolate,
};

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
mod car_forward_dodge;
mod car_powerslide_turn;
pub mod car_single_jump;
mod collision;
mod math;
