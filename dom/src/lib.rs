/*!
Rust translation of some code from DomNomNom's [RocketBot].

[RocketBot]: https://github.com/DomNomNom/RocketBot
*/

#![cfg_attr(feature = "strict", deny(warnings))]

extern crate common;
extern crate nalgebra;

pub use student_agents::get_pitch_yaw_roll;

mod student_agents;
