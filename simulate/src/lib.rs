#![cfg_attr(feature = "strict", deny(warnings))]

extern crate nalgebra;

pub use car1d::Car1D;
pub use car_aerial_60deg::CarAerial60Deg;
pub use math::linear_interpolate;

mod car1d;
mod car_aerial_60deg;
pub mod car_single_jump;
mod math;
pub mod rl;
mod tables;
