use eeg::{color, Drawable, EEG};
use mechanics::simple_yaw_diff;
use nalgebra::Vector2;
use rlbot;
use simulate::{linear_interpolate, rl};
use std::f32::consts::PI;
use utils::{one_v_one, ExtendPhysics};

pub fn drive_towards(
    packet: &rlbot::ffi::LiveDataPacket,
    eeg: &mut EEG,
    target_loc: Vector2<f32>,
) -> rlbot::ffi::PlayerInput {
    let (me, _enemy) = one_v_one(packet);

    let yaw_diff = simple_yaw_diff(&me.Physics, target_loc);
    let steer = yaw_diff.max(-1.0).min(1.0) * 2.0;

    eeg.draw(Drawable::print(stringify!(drive_towards), color::YELLOW));
    eeg.draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));

    let handbrake_cutoff = linear_interpolate(
        &[0.0, rl::CAR_NORMAL_SPEED],
        &[PI * 0.25, PI * 0.75],
        me.Physics.vel().norm(),
    );

    rlbot::ffi::PlayerInput {
        Throttle: 1.0,
        Steer: steer,
        Handbrake: yaw_diff.abs() >= handbrake_cutoff,
        ..Default::default()
    }
}
