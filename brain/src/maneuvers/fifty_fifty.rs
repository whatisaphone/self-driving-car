use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::BlitzToLocation;
use mechanics::QuickJumpAndDodge;
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use utils::{my_goal_center, one_v_one, ExtendPhysics};

pub struct FiftyFifty;

impl FiftyFifty {
    pub fn new() -> FiftyFifty {
        FiftyFifty
    }
}

impl Behavior for FiftyFifty {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        // Get between the ball and our goal
        let target_loc =
            intercept.ball_loc + (my_goal_center() - intercept.ball_loc).normalize() * 200.0;
        let target_dist = (target_loc - me.Physics.loc()).norm();

        eeg.draw(Drawable::print("FiftyFifty", color::YELLOW));
        eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostCar(target_loc, me.Physics.rot()));

        if target_dist >= 300.0 {
            // TODO: this is not how this works…
            let mut child = BlitzToLocation::new(target_loc);
            child.execute(packet, eeg)
        } else {
            Action::call(QuickJumpAndDodge::begin(packet))
        }
    }
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::FiftyFifty;
    use nalgebra::Vector3;

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            FiftyFifty,
            TestScenario {
                car_loc: Vector3::new(256.0, -3839.98, 17.01),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }
}
