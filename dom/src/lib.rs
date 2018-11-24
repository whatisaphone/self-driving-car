/*!
Rust translation of some code from DomNomNom's [RocketBot].

[RocketBot]: https://github.com/DomNomNom/RocketBot
*/

#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
extern crate nalgebra;

pub use student_agents::get_pitch_yaw_roll;
pub use vector_math::to_rotation_matrix;

mod student_agents;
mod vector_math;
