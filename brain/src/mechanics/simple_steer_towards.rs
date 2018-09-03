use collect::ExtendRotation3;
use nalgebra::Vector3;
use rlbot;
use utils::{ExtendF32, ExtendPhysics, ExtendVector3};

pub fn simple_steer_towards(car: &rlbot::Physics, target_loc: Vector3<f32>) -> f32 {
    simple_yaw_diff(car, &target_loc).max(-1.0).min(1.0) * 2.0
}

pub fn simple_yaw_diff(car: &rlbot::Physics, target_loc: &Vector3<f32>) -> f32 {
    let target_yaw = car.loc().angle_to(&target_loc);
    (target_yaw - car.rot().yaw()).normalize_angle()
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
        fn name(&self) -> &'static str {
            "SimpleSteerTowardsBall"
        }

        fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
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
