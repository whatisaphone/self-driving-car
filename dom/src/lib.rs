/*!
Rust translation of some code from DomNomNom's [RocketBot].

[RocketBot]: https://github.com/DomNomNom/RocketBot
*/

#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![warn(clippy::all)]

pub use crate::{student_agents::get_pitch_yaw_roll, vector_math::to_rotation_matrix};

mod student_agents;
mod vector_math;
