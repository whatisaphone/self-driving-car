use eeg::{color, Drawable, EEG};
use maneuvers::Maneuver;
use mechanics::GroundAccelToLoc;
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use utils::{one_v_one, ExtendPhysics};

pub struct FiftyFifty;

impl Maneuver for FiftyFifty {
    fn execute(&self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput {
        eeg.draw(Drawable::print("FiftyFifty", color::YELLOW));

        let (me, _enemy) = one_v_one(packet);
        let (intercept_time, intercept_loc) = estimate_intercept_car_ball(&me, &packet.GameBall);

        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept_time),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostCar(intercept_loc, me.Physics.rot()));

        let child =
            GroundAccelToLoc::new(intercept_loc, packet.GameInfo.TimeSeconds + intercept_time);
        child.execute(packet, eeg)
    }
}

#[cfg(test)]
mod tests {
    use maneuvers::fifty_fifty::FiftyFifty;
    use nalgebra::Vector3;
    use tests::helpers::TestRunner;
    use tests::helpers::TestScenario;

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            Box::new(FiftyFifty),
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
