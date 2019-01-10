// A nice idiot-friendly site with the equations:
//
// http://zonalandeducation.com/mstm/physics/mechanics/kinematics/EquationsForAcceleratedMotion/EquationsForAcceleratedMotion.htm

use crate::math::quadratic;
use nalgebra::Vector3;

/// Calculate displacement and velocity given a set of initial conditions and a
/// constant acceleration.
pub fn kinematic(v_0: Vector3<f32>, a: Vector3<f32>, t: f32) -> (Vector3<f32>, Vector3<f32>) {
    let d = v_0 * t + 0.5 * a * t * t;
    let v_f = v_0 + a * t;
    (d, v_f)
}

/// Calculate time given distance traveled, initial velocity, and constant
/// acceleration.
pub fn kinematic_time(d: f32, v_0: f32, a: f32) -> Option<f32> {
    let [x, y] = quadratic(0.5 * a, v_0, -d)?;
    // Choose the minimum positive result.
    match (x > 0.0, y > 0.0) {
        (false, false) => None,
        (false, true) => Some(y),
        (true, false) => Some(x),
        (true, true) => Some(x.min(y)),
    }
}
