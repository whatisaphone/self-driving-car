use behavior::{Action, Behavior};
use common::{prelude::*, rl};
use eeg::{color, Drawable};
use mechanics::{simple_steer_towards, QuickJumpAndDodge};
use nalgebra::Point2;
use rlbot;
use std::f32::consts::PI;
use strategy::Context;

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
        let distance = (me.Physics.locp().to_2d() - self.target_loc).norm();
        let speed = me.Physics.vel().norm();

        let steer = simple_steer_towards(&me.Physics, self.target_loc.coords);

        ctx.eeg.draw(Drawable::ghost_car_ground(
            self.target_loc.coords,
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

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::blitz_to_location::BlitzToLocation;
    use nalgebra::{Point2, Vector3};

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            BlitzToLocation {
                target_loc: Point2::origin(),
            },
            TestScenario {
                car_loc: Vector3::new(256.0, -3839.98, 17.01),
                ..Default::default()
            },
        );

        test.sleep_millis(3000);

        // Assert that we touched and moved the ball.
        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.Location);
        assert!(packet.GameBall.Physics.Location.Y.abs() > 1.0);
    }
}
