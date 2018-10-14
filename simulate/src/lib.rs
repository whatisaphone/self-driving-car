#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
extern crate nalgebra;
extern crate ncollide3d;
extern crate rlbot;

pub use car::Car;
pub use car1d::Car1D;
pub use car_aerial_60deg::CarAerial60Deg;
pub use collision::ball_car_distance;
pub use math::linear_interpolate;

mod car;
mod car1d;
mod car_aerial_60deg;
pub mod car_single_jump;
mod collision;
mod math;
pub mod rl;
mod tables;
