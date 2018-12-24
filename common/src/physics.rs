//! Various mathematical truths about Rocket League.

use lazy_static::lazy_static;
use nalgebra::{Unit, UnitComplex, UnitQuaternion, Vector2, Vector3};

lazy_static! {
    pub static ref CAR_LOCAL_FORWARD_AXIS: Unit<Vector3<f32>> = Vector3::x_axis();
    pub static ref CAR_LOCAL_FORWARD_AXIS_2D: Unit<Vector2<f32>> = Vector2::x_axis();
    pub static ref CAR_LOCAL_RIGHT_AXIS: Unit<Vector3<f32>> = Vector3::y_axis();
    pub static ref CAR_LOCAL_RIGHT_AXIS_2D: Unit<Vector2<f32>> = Vector2::y_axis();
    pub static ref CAR_LOCAL_ROOF_AXIS: Unit<Vector3<f32>> = Vector3::z_axis();
}

/// Returns the forward axis in world coordinates of a car with the given
/// rotation.
pub fn car_forward_axis(rot: UnitQuaternion<f32>) -> Unit<Vector3<f32>> {
    rot * *CAR_LOCAL_FORWARD_AXIS
}

/// Returns the forward axis in 2D world coordinates of a car with the given 2D
/// rotation.
pub fn car_forward_axis_2d(rot: UnitComplex<f32>) -> Unit<Vector2<f32>> {
    rot * *CAR_LOCAL_FORWARD_AXIS_2D
}

/// Returns the right axis in world coordinates of a car with the given
/// rotation.
pub fn car_right_axis(rot: UnitQuaternion<f32>) -> Unit<Vector3<f32>> {
    rot * *CAR_LOCAL_RIGHT_AXIS
}

/// Returns the right axis in 2D world coordinates of a car with the given 2D
/// rotation.
pub fn car_right_axis_2d(rot: UnitComplex<f32>) -> Unit<Vector2<f32>> {
    rot * *CAR_LOCAL_RIGHT_AXIS_2D
}

/// Returns a unit vector in world coordinates in the direction of the roof of a
/// car with the given rotation.
pub fn car_roof_axis(rot: UnitQuaternion<f32>) -> Unit<Vector3<f32>> {
    rot * *CAR_LOCAL_ROOF_AXIS
}
