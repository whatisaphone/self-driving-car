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

pub fn kinematic_solve_for_t(d: f32, v_0: f32, a: f32) -> Option<f32> {
    quadratic(0.5 * a, v_0, -d).map(|[_, t]| t)
}
