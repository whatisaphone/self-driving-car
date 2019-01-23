use common::prelude::*;
use nalgebra::UnitQuaternion;

pub fn rotator(r: UnitQuaternion<f32>) -> rlbot::state::RotatorPartial {
    let (pitch, yaw, roll) = r.to_rotation_matrix().to_unreal_angles();
    rlbot::state::RotatorPartial::new()
        .pitch(pitch)
        .yaw(yaw)
        .roll(roll)
}
