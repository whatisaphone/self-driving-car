use eeg::{Drawable, EEG};
use mechanics::simple_yaw_diff;
use nalgebra::Vector2;
use rlbot;
use std::f32::consts::PI;
use utils::{one_v_one, ExtendPhysics};

pub fn drive_towards(
    packet: &rlbot::LiveDataPacket,
    eeg: &mut EEG,
    target_loc: Vector2<f32>,
) -> rlbot::PlayerInput {
    let (me, _enemy) = one_v_one(packet);

    let yaw_diff = simple_yaw_diff(&me.Physics, target_loc);
    let steer = yaw_diff.max(-1.0).min(1.0) * 2.0;

    eeg.draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));

    rlbot::PlayerInput {
        Throttle: 1.0,
        Steer: steer,
        Handbrake: yaw_diff.abs() >= PI / 2.0,
        ..Default::default()
    }
}
