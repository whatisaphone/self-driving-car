use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use eeg::{color, Drawable, EEG};
use mechanics::{simple_steer_towards, QuickJumpAndDodge};
use nalgebra::Vector2;
use rlbot;
use simulate::rl;
use std::f32::consts::PI;
use utils::{my_car, ExtendPhysics, ExtendVector3};

pub struct BlitzToLocation {
    target_loc: Vector2<f32>,
}

impl BlitzToLocation {
    pub fn new(target_loc: Vector2<f32>) -> BlitzToLocation {
        BlitzToLocation { target_loc }
    }
}

impl Behavior for BlitzToLocation {
    fn name(&self) -> &'static str {
        stringify!(BlitzToLocation)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);
        let distance = (me.Physics.loc().to_2d() - self.target_loc).norm();
        let speed = me.Physics.vel().norm();

        let steer = simple_steer_towards(&me.Physics, self.target_loc);

        eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
            me.Physics.rot(),
        ));
        eeg.draw(Drawable::print(
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
            return Action::Yield(rlbot::PlayerInput {
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
                return Action::Call(Box::new(QuickJumpAndDodge::begin(packet)));
            }
        }

        Action::Yield(rlbot::PlayerInput {
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
    use nalgebra::{Vector2, Vector3};

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            BlitzToLocation {
                target_loc: Vector2::zeros(),
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
