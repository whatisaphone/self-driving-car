use crate::{
    behavior::{
        higher_order::Chain,
        movement::{simple_steer_towards, QuickJumpAndDodge, Yielder},
    },
    eeg::Drawable,
    strategy::{Action, Behavior, Context},
};
use common::{prelude::*, rl, Distance};
use nalgebra::Point2;
use nameof::name_of_type;
use std::f32::consts::PI;
use vec_box::vec_box;

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
        name_of_type!(BlitzToLocation)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let me = ctx.me();
        let distance = (me.Physics.loc_2d() - self.target_loc).norm();
        let speed = me.Physics.vel().norm();

        let steer = simple_steer_towards(&me.Physics, self.target_loc);

        ctx.eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
            me.Physics.rot(),
        ));
        ctx.eeg.print_value("distance", Distance(distance));

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
            return Action::Yield(common::halfway_house::PlayerInput {
                Throttle: 1.0,
                Steer: steer,
                Boost: true,
                ..Default::default()
            });
        }

        // Should we flip?
        if me.OnGround
            && me.Physics.rot().pitch().to_degrees() < 1.0
            && (900.0 <= speed && speed < 2200.0)
        {
            // In theory this value should be 1.25, but leave some leeway for recovery.
            let assumed_flip_duration = 1.75;
            let dodge_speed = me.Physics.vel().norm() + rl::DODGE_FORWARD_IMPULSE;
            let flip_dist = dodge_speed * assumed_flip_duration;
            if (distance > flip_dist && steer.abs() < PI / 24.0)
                || (distance > flip_dist * 1.5 && steer.abs() < PI / 8.0)
            {
                return Action::tail_call(Chain::new(self.priority(), vec_box![
                    // Give a bit of time to stabilize in case we just landed.
                    Yielder::new(0.1, Default::default()),
                    QuickJumpAndDodge::new()
                ]));
            }
        }

        Action::Yield(common::halfway_house::PlayerInput {
            Throttle: 1.0,
            Steer: steer,
            ..Default::default()
        })
    }
}
