use common::{
    kinematics::{kinematic, kinematic_solve_for_t},
    physics::car_roof_axis,
    rl,
};
use nalgebra::{UnitQuaternion, Vector3};

/// With the given car rotation, how long would it take to jump a certain
/// distance along the car's roof axis?
///
/// Note that this can return degenerate results, e.g., if the car is sideways,
/// you can jump an infinite distance to the side if you're willing to wait long
/// enough.
pub fn jump_duration(rot: &UnitQuaternion<f32>, target_dist: f32) -> Option<f32> {
    // A jump has two stages:
    //
    // 1. A period of constant force for up to 0.2s while the jump button is held
    //    down.
    // 2. A period of freefall.

    // First step: will we reach the distance during the initial 0.2 seconds of
    // force?
    //
    // Solve the kinematic equation for t.
    //
    //     d = d0 + v0 * t + 1/2 * a * t^2
    //
    // e.g.
    //
    //     (a)t^2 + (2 * v0)t + (2 * d0 - 2 * d)

    let axis = car_roof_axis(*rot).unwrap();

    let v_0 = axis * rl::CAR_JUMP_IMPULSE_SPEED;
    let a = axis * rl::CAR_JUMP_ACCEL + Vector3::z() * rl::GRAVITY;
    if let Some(t) = kinematic_solve_for_t(target_dist, v_0.dot(&axis), a.dot(&axis)) {
        if t < rl::CAR_JUMP_FORCE_TIME {
            return Some(t);
        }
    }

    // Next step: will we reach it at all? Simulate the full 0.2s of force, then
    // solve the kinematic equation while coasting for t.

    let (d_0, v_0) = kinematic(v_0, a, rl::CAR_JUMP_FORCE_TIME);
    let a = Vector3::z() * rl::GRAVITY;
    if let Some(t) =
        kinematic_solve_for_t(target_dist - d_0.dot(&axis), v_0.dot(&axis), a.dot(&axis))
    {
        return Some(rl::CAR_JUMP_FORCE_TIME + t);
    }

    // If there's still no solution, it isn't possible to jump that high.

    None
}

#[cfg(test)]
mod tests {
    use crate::car_jump::jump_duration;
    use nalgebra::UnitQuaternion;

    #[test]
    fn test_jump_duration() {
        let flat = UnitQuaternion::identity();
        assert_eq!(jump_duration(&flat, 10.0).unwrap(), 0.03279536);
        assert_eq!(jump_duration(&flat, 100.0).unwrap(), 0.25872213);
        assert_eq!(jump_duration(&flat, 200.0).unwrap(), 0.5807926);
        assert_eq!(jump_duration(&flat, 220.0).unwrap(), 0.70065045);
        assert_eq!(jump_duration(&flat, 250.0), None);
    }
}
