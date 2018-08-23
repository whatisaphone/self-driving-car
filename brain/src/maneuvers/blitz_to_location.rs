use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::misc::simple_steer_towards;
use mechanics::QuickJumpAndDodge;
use nalgebra::Vector3;
use rlbot;
use std::f32::consts::PI;
use utils::{my_car, ExtendPhysics};

pub struct BlitzToLocation {
    target_loc: Vector3<f32>,
}

impl BlitzToLocation {
    pub fn new(target_loc: Vector3<f32>) -> BlitzToLocation {
        BlitzToLocation { target_loc }
    }
}

impl Behavior for BlitzToLocation {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("BlitzToLocation", color::YELLOW));

        let me = my_car(packet);
        let distance = (me.Physics.loc() - self.target_loc).norm();
        let speed = me.Physics.vel().norm();

        eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));

        let steer = simple_steer_towards(&me.Physics, self.target_loc);

        // Should we boost?
        if distance > 1000.0
            && steer.abs() < PI / 2.0
            && me.OnGround
            && me.Boost > 0
            // After ~1500 (very unscientific number), we can hit max speed quicker by flipping. After
            // ~2000 (same), it's probably not worth losing wheel contact (and thus
            // momentarily losing agility).
            && (speed < 1500.0 || (2000.0 <= speed && speed < 2290.0))
        {
            return Action::Yield(rlbot::PlayerInput {
                Throttle: 1.0,
                Steer: steer,
                Boost: true,
                ..Default::default()
            });
        }

        // Should we flip?
        if distance > 2000.0
            && steer.abs() < PI / 8.0
            && me.OnGround
            && (800.0 <= speed && speed < 2200.0)
        {
            return Action::Call(Box::new(QuickJumpAndDodge::begin(packet)));
        }

        Action::Yield(rlbot::PlayerInput {
            Throttle: 1.0,
            Steer: steer,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use maneuvers::blitz_to_location::BlitzToLocation;
    use nalgebra::Vector3;
    use tests::helpers::TestRunner;
    use tests::helpers::TestScenario;

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            BlitzToLocation {
                target_loc: Vector3::zeros(),
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
