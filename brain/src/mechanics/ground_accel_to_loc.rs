use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::simple_steer_towards;
use nalgebra::Vector3;
use rlbot;
use simulate::{rl, Car1D};
use std::f32::consts::PI;
use utils::{my_car, ExtendPhysics};

pub struct GroundAccelToLoc {
    target_loc: Vector3<f32>,
    target_time: f32,
}

impl GroundAccelToLoc {
    pub fn new(target_loc: Vector3<f32>, target_time: f32) -> GroundAccelToLoc {
        GroundAccelToLoc {
            target_loc,
            target_time,
        }
    }
}

impl Behavior for GroundAccelToLoc {
    fn name(&self) -> &'static str {
        "GroundAccelToLoc"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);
        let distance = (me.Physics.loc() - self.target_loc).norm();
        let time_remaining = self.target_time - packet.GameInfo.TimeSeconds;

        eeg.draw(Drawable::print(self.name(), color::YELLOW));
        eeg.draw(Drawable::GhostCar(self.target_loc, me.Physics.rot()));
        eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));

        // This behavior currently just operates in 2D
        if !GetToFlatGround::on_flat_ground(packet) {
            return Action::call(GetToFlatGround::new());
        }

        let mut result = rlbot::PlayerInput::default();
        result.Steer = simple_steer_towards(&me.Physics, self.target_loc);
        if !estimate_approach(&me, distance, time_remaining - 2.0 / 120.0) {
            result.Throttle = 1.0;
            if me.OnGround
                && result.Steer.abs() < PI / 4.0
                && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED
                && me.Boost > 0
            {
                result.Boost = true;
            }
        }

        Action::Yield(result)
    }
}

/// Starting at `origin`, if we go pedal to the metal for `time` seconds, will
/// we have traveled `distance`?
fn estimate_approach(car: &rlbot::PlayerInfo, distance: f32, time: f32) -> bool {
    const DT: f32 = 1.0 / 60.0;

    let mut t = 1.5 / 120.0; // Start a few ticks later to compensate for input lag.
    let mut sim_car = Car1D::new(car.Physics.vel().norm()).with_boost(car.Boost);

    while t < time {
        t += DT;
        sim_car.step(DT, 1.0, true);

        if sim_car.distance_traveled() >= distance {
            return true;
        }
    }

    false
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use mechanics::GroundAccelToLoc;
    use nalgebra::Vector3;
    use utils::{ExtendPhysics, ExtendVector3};

    // This test is ignored because it's finicky and not quite accurate. The
    // issue seems to be that there is more input lag for throttle than for
    // boost? Weird stuff. Anyhow, this test is a nice ideal at least :)
    #[test]
    #[ignore]
    fn verify_arrival_time() {
        let cases = [(-200.0, 500.0, 0), (100.0, 600.0, 50)];
        for &(x, y, boost) in cases.iter() {
            let target_loc = Vector3::new(x, y, 0.0);
            let test = TestRunner::start2(
                TestScenario {
                    ball_loc: Vector3::new(2000.0, 0.0, 0.0),
                    boost,
                    ..Default::default()
                },
                move |p| GroundAccelToLoc::new(target_loc, p.GameInfo.TimeSeconds + 2.0),
            );

            test.sleep_millis(2000);

            let packet = test.sniff_packet();
            let diff = (packet.GameCars[0].Physics.loc() - target_loc)
                .to_2d()
                .norm();
            println!("target loc: {:.?}", target_loc);
            println!("car loc: {:.?}", packet.GameCars[0].Physics.loc());
            println!("diff: {:.0}", diff);
            assert!(diff.abs() < 20.0);
        }
    }
}
