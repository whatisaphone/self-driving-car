#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
#[macro_use]
extern crate lazy_static;
extern crate nalgebra;
extern crate ncollide3d;
extern crate oven;
extern crate rlbot;

pub use crate::{
    car::{Car, CarSimulateError},
    car1d::Car1D,
    car1dv2::Car1Dv2,
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
mod car1dv2;
mod car_forward_dodge;
mod car_powerslide_turn;
pub mod car_single_jump;
mod collision;
mod math;
