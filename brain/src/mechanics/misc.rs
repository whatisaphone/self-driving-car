use collect::ExtendRotation3;
use nalgebra::Vector3;
use rlbot;
use utils::geometry::normalize_angle;
use utils::ExtendPhysics;

pub fn simple_steer_towards(car: &rlbot::Physics, target_loc: Vector3<f32>) -> f32 {
    let diff = target_loc - car.loc();
    let target_yaw = f32::atan2(diff.y, diff.x);
    let result = normalize_angle(target_yaw - car.rot().yaw());
    result.max(-1.0).min(1.0)
}
