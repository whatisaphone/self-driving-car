use collect::ExtendRotation3;
use nalgebra::Vector3;
use rlbot;
use utils::geometry::{ExtendF32, ExtendVector3};
use utils::ExtendPhysics;

pub fn simple_steer_towards(car: &rlbot::Physics, target_loc: Vector3<f32>) -> f32 {
    let target_yaw = car.loc().angle_to(&target_loc);
    let result = (target_yaw - car.rot().yaw()).normalize_angle();
    result.max(-1.0).min(1.0) * 2.0
}

#[cfg(test)]
mod integration_tests {
    use behavior::{Action, Behavior};
    use collect::ExtendRotation3;
    use eeg::{color, Drawable, EEG};
    use integration_tests::helpers::{TestRunner, TestScenario};
    use mechanics::simple_steer_towards;
    use nalgebra::{Rotation3, Vector3};
    use rlbot;
    use utils::ExtendPhysics;

    struct SimpleSteerTowardsBall;

    impl Behavior for SimpleSteerTowardsBall {
        fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
            eeg.draw(Drawable::print("SimpleSteerTowardsBall", color::YELLOW));

            let me = packet.GameCars[0];
            let ball = packet.GameBall;
            Action::Yield(rlbot::PlayerInput {
                Throttle: 1.0,
                Steer: simple_steer_towards(&me.Physics, ball.Physics.loc()),
                ..Default::default()
            })
        }
    }

    #[test]
    fn steer_direction() {
        let test = TestRunner::start(
            SimpleSteerTowardsBall,
            TestScenario {
                ball_loc: Vector3::new(2800.0, -2800.0, 0.0),
                car_loc: Vector3::new(3800.0, -2400.0, 0.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 2.8, 0.0),
                ..Default::default()
            },
        );

        test.sleep_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.loc().x < 2700.0);
    }
}
