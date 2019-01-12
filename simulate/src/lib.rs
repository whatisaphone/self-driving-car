#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]
#![allow(clippy::unreadable_literal)]

pub use crate::{
    car::{Car, CarSimulateError},
    car1d::Car1D,
    car_forward_dodge::{CarForwardDodge, CarForwardDodge1D},
    car_powerslide_turn::{CarPowerslideTurn, CarPowerslideTurnBlueprint},
    collision::ball_car_distance,
    math::linear_interpolate,
};

mod car;
mod car1d;
mod car_forward_dodge;
pub mod car_jump;
mod car_powerslide_turn;
pub mod car_single_jump;
mod collision;
mod math;
