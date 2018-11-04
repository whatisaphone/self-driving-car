use behavior::{Action, Behavior};
use common::prelude::*;
use eeg::{color, Drawable, EEG};
use mechanics::simple_yaw_diff;
use nalgebra::{Point2, Vector2};
use rlbot;
use simulate::{linear_interpolate, rl};
use std::f32::consts::PI;
use strategy::Context;
use utils::one_v_one;

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
        &[PI * 0.25, PI * 0.50],
        me.Physics.vel().norm(),
    );

    rlbot::ffi::PlayerInput {
        Throttle: 1.0,
        Steer: steer,
        Handbrake: yaw_diff.abs() >= handbrake_cutoff,
        ..Default::default()
    }
}

/// A naive driving behavior that doesn't even know when it's arrived. Must be
/// combined with `TimeLimit` or something else to bring back sanity.
pub struct DriveTowards {
    target_loc: Point2<f32>,
}

impl DriveTowards {
    pub fn new(target_loc: Point2<f32>) -> Self {
        Self { target_loc }
    }
}

impl Behavior for DriveTowards {
    fn name(&self) -> &str {
        stringify!(DriveTowards)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        Action::Yield(drive_towards(ctx.packet, ctx.eeg, self.target_loc.coords))
    }
}
