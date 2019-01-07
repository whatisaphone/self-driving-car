use nalgebra::{Point3, Vector3};

/// Calculate displacement and velocity given a set of initial conditions and a
/// constant acceleration.
pub fn kinematic(
    d_0: Point3<f32>,
    v_0: Vector3<f32>,
    a: Vector3<f32>,
    t: f32,
) -> (Point3<f32>, Vector3<f32>) {
    let d = v_0 * t + 0.5 * a * t * t;
    let v_f = v_0 + a * t;
    (d_0 + d, v_f)
}
