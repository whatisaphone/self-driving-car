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
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("BounceShot", color::YELLOW));

        let (me, _enemy) = one_v_one(packet);
        let (intercept_time, intercept_loc) = estimate_intercept_car_ball(&me, &packet.GameBall);

        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept_time),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostCar(intercept_loc, me.Physics.rot()));

        // TODO: this is not how this works…
        let mut child =
            GroundAccelToLoc::new(intercept_loc, packet.GameInfo.TimeSeconds + intercept_time);
        child.execute(packet, eeg)
    }
}

#[cfg(test)]
mod tests {
    use maneuvers::bounce_shot::BounceShot;
    use nalgebra::Vector3;
    use tests::helpers::TestRunner;
    use tests::helpers::TestScenario;

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
