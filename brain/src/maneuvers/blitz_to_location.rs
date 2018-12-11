use crate::{
    behavior::{Action, Behavior},
    eeg::{color, Drawable},
    mechanics::{simple_steer_towards, QuickJumpAndDodge},
    strategy::Context,
};
use common::{prelude::*, rl};
use nalgebra::Point2;
use rlbot;
use std::f32::consts::PI;

pub struct BlitzToLocation {
    target_loc: Point2<f32>,
}

impl BlitzToLocation {
    pub fn new(target_loc: Point2<f32>) -> BlitzToLocation {
        BlitzToLocation { target_loc }
    }
}

impl Behavior for BlitzToLocation {
    fn name(&self) -> &str {
        stringify!(BlitzToLocation)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();
        let distance = (me.Physics.loc_2d() - self.target_loc).norm();
        let speed = me.Physics.vel().norm();

        let steer = simple_steer_towards(&me.Physics, self.target_loc);

        ctx.eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
            me.Physics.rot(),
        ));
        ctx.eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));

        // Should we boost?
        if distance > 1000.0
            && me.OnGround
            && steer.abs() < PI / 4.0
            // After ~1500 (very unscientific number), we can hit max speed
            // quicker by flipping. After ~2000 (same), it's probably not worth
            // losing wheel contact (and thus agility).
            && (speed < 1500.0 || (2000.0 <= speed && speed < rl::CAR_ALMOST_MAX_SPEED))
            && me.Boost > 0
        {
            return Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Steer: steer,
                Boost: true,
                ..Default::default()
            });
        }

        // Should we flip?
        if me.OnGround
            && me.Physics.rot().pitch().to_degrees() < 1.0
            && (800.0 <= speed && speed < 2200.0)
        {
            let flip_dist = me.Physics.vel().norm() * 3.0;
            if (distance > flip_dist && steer.abs() < PI / 24.0)
                || (distance > flip_dist * 1.5 && steer.abs() < PI / 8.0)
            {
                return Action::Call(Box::new(QuickJumpAndDodge::begin(ctx.packet)));
            }
        }

        Action::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: steer,
            ..Default::default()
        })
    }
}
