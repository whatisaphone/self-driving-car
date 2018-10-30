use common::prelude::*;
use nalgebra::Vector2;
use rlbot;
use utils::{ExtendF32, ExtendVector2, ExtendVector3};

pub fn simple_steer_towards(car: &rlbot::ffi::Physics, target_loc: Vector2<f32>) -> f32 {
    simple_yaw_diff(car, target_loc).max(-1.0).min(1.0) * 2.0
}

pub fn simple_yaw_diff(car: &rlbot::ffi::Physics, target_loc: Vector2<f32>) -> f32 {
    let target_yaw = car.loc().to_2d().angle_to(target_loc);
    (target_yaw - car.rot().yaw()).normalize_angle()
}

#[cfg(test)]
mod integration_tests {
    use behavior::{Action, Behavior};
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use mechanics::simple_steer_towards;
    use nalgebra::{Rotation3, Vector3};
    use rlbot;
    use strategy::Context;
    use utils::ExtendVector3;

    struct SimpleSteerTowardsBall;

    impl Behavior for SimpleSteerTowardsBall {
        fn name(&self) -> &'static str {
            stringify!(SimpleSteerTowardsBall)
        }

        fn execute2(&mut self, ctx: &mut Context) -> Action {
            let me = ctx.me();
            let ball = ctx.packet.GameBall;
            Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Steer: simple_steer_towards(&me.Physics, ball.Physics.loc().to_2d()),
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
