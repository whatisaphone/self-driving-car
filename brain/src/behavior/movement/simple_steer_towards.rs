use crate::utils::geometry::ExtendF32;
use common::prelude::*;
use nalgebra::Point2;

pub fn simple_steer_towards(car: &rlbot::ffi::Physics, target_loc: Point2<f32>) -> f32 {
    simple_yaw_diff(car, target_loc).max(-1.0).min(1.0) * 2.0
}

pub fn simple_yaw_diff(car: &rlbot::ffi::Physics, target_loc: Point2<f32>) -> f32 {
    let target_yaw = car.loc_2d().negated_difference_and_angle_to(target_loc);
    (target_yaw - car.rot().yaw()).normalize_angle()
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::movement::simple_steer_towards,
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::{Action, Behavior, Context},
    };
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3};
    use nameof::name_of_type;

    struct SimpleSteerTowardsBall;

    impl Behavior for SimpleSteerTowardsBall {
        fn name(&self) -> &'static str {
            name_of_type!(SimpleSteerTowardsBall)
        }

        fn execute(&mut self, ctx: &mut Context<'_>) -> Action {
            let me = ctx.me();
            let ball = ctx.packet.GameBall;
            Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Steer: simple_steer_towards(&me.Physics, ball.Physics.loc_2d()),
                ..Default::default()
            })
        }
    }

    #[test]
    fn steer_direction() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2800.0, -2800.0, 0.0),
                car_loc: Point3::new(3800.0, -2400.0, 0.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 2.8, 0.0),
                ..Default::default()
            })
            .behavior(SimpleSteerTowardsBall)
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.loc().x < 2700.0);
    }
}
