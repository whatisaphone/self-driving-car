// Source: https://github.com/DomNomNom/RocketBot/blob/c7553453acf4d0c3ff286cc2d9b44f9cc3a784b4/student_agents.py

use common::prelude::*;
use nalgebra::{Unit, Vector3};

/// Calculate (pitch, yaw, roll) inputs to align the car's rotation with the
/// given `forward` and `up` vectors.
pub fn get_pitch_yaw_roll(
    car: &rlbot::ffi::PlayerInfo,
    forward: Unit<Vector3<f32>>,
    up: Unit<Vector3<f32>>,
) -> (f32, f32, f32) {
    let desired_facing_angular_vel = -car.Physics.forward_axis().cross(&forward);
    let desired_up_angular_vel = -car.Physics.roof_axis().cross(&up);

    let mut pitch = desired_facing_angular_vel.dot(&car.Physics.right_axis());
    let yaw = -desired_facing_angular_vel.dot(&car.Physics.roof_axis());
    let mut roll = desired_up_angular_vel.dot(&car.Physics.forward_axis());

    let mut pitch_vel = car.Physics.ang_vel().dot(&car.Physics.right_axis());
    let yaw_vel = -car.Physics.ang_vel().dot(&car.Physics.roof_axis());
    let roll_vel = car.Physics.ang_vel().dot(&car.Physics.forward_axis());

    // avoid getting stuck in directly-opposite states
    if car.Physics.roof_axis().dot(&up) < -0.8 && car.Physics.forward_axis().dot(&forward) > 0.8 {
        if roll == 0.0 {
            roll = 1.0;
        }
        roll *= 1e10;
    }
    if car.Physics.forward_axis().dot(&forward) < -0.8 {
        if pitch == 0.0 {
            pitch = 1.0;
        }
        pitch *= 1e10;
    }

    if car.Physics.forward_axis().dot(&forward) < 0.0 {
        pitch_vel *= -1.0;
    }

    // PID control to stop overshooting.
    let mut roll = 3.0 * roll + 0.30 * roll_vel;
    let yaw = 3.0 * yaw + 0.70 * yaw_vel;
    let pitch = 3.0 * pitch + 0.90 * pitch_vel;

    // only start adjusting roll once we're roughly facing the right way
    if car.Physics.forward_axis().dot(&forward) < 0.0 {
        roll = 0.0;
    }

    (pitch, yaw, roll)
}
