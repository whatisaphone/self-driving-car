use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::GroundAccelToLoc;
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use utils::{one_v_one, ExtendPhysics};

pub struct BounceShot;

impl BounceShot {
    pub fn new() -> BounceShot {
        BounceShot
    }
}

impl Behavior for BounceShot {
    fn name(&self) -> &'static str {
        "BounceShot"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        eeg.draw(Drawable::print(self.name(), color::YELLOW));
        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostCar(intercept.car_loc, me.Physics.rot()));

        // TODO: this is not how this works…
        let mut child = GroundAccelToLoc::new(
            intercept.car_loc,
            packet.GameInfo.TimeSeconds + intercept.time,
        );
        child.execute(packet, eeg)
    }
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::bounce_shot::BounceShot;
    use nalgebra::Vector3;

    #[test]
    fn normal() {
        let test = TestRunner::start(
            BounceShot,
            TestScenario {
                ball_loc: Vector3::new(-2000.0, 2000.0, 500.0),
                ball_vel: Vector3::new(1000.0, 0.0, 0.0),
                car_loc: Vector3::new(0.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, -200.0, 0.0),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn slow_no_boost() {
        let test = TestRunner::start(
            BounceShot,
            TestScenario {
                ball_loc: Vector3::new(-2000.0, 2000.0, 1000.0),
                ball_vel: Vector3::new(500.0, 0.0, 0.0),
                car_loc: Vector3::new(0.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, -200.0, 0.0),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(6000);

        assert!(test.has_scored());
    }
}
