use behavior::{Action, Behavior};
use common::prelude::*;
use eeg::{color, Drawable, EEG};
use mechanics::{simple_steer_towards, DriveLocTimeDecelerate};
use nalgebra::Vector2;
use predict::estimate_intercept_car_ball_2;
use rlbot;
use utils::{enemy_goal_center, one_v_one};

pub struct CarryFromBounce;

impl CarryFromBounce {
    pub fn new() -> CarryFromBounce {
        CarryFromBounce
    }
}

impl Behavior for CarryFromBounce {
    fn name(&self) -> &str {
        stringify!(CarryFromBounce)
    }

    fn execute(&mut self, packet: &rlbot::ffi::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |_t, loc, vel| {
            120.0 <= loc.z && loc.z < 140.0 && vel.z < 0.0
        });

        let theta1 = enemy_goal_center().angle_to(intercept.ball_loc.to_2d());
        let theta2 = Vector2::zeros().angle_to(packet.GameBall.Physics.vel().to_2d());
        let theta = (theta1 + theta2 * 0.2) / 1.2;
        let target_loc =
            intercept.ball_loc.to_2d() + Vector2::new(theta.cos(), theta.sin()) * 200.0;
        let target_dist = (target_loc - me.Physics.loc().to_2d()).norm();

        eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostBall(intercept.ball_loc));

        if !me.OnGround {
            eeg.draw(Drawable::print("I'm scared", color::RED));
            return Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Steer: simple_steer_towards(&me.Physics, enemy_goal_center()),
                ..Default::default()
            });
        }

        // TODO: this is not how this works…
        DriveLocTimeDecelerate::new(
            target_loc,
            me.Physics.vel().norm() / 2.0,
            packet.GameInfo.TimeSeconds + intercept.time,
        )
        .execute(packet, eeg)
    }
}

//fn cut_loc(packet: &rlbot::ffi::LiveDataPacket) -> Vector3<f32> {
//}

#[cfg(test)]
mod integration_tests {
    use behavior::carry_from_bounce::CarryFromBounce;
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn quick_pickup() {
        let test = TestRunner::start(
            CarryFromBounce::new(),
            TestScenario {
                ball_loc: Vector3::new(2087.2695, -3514.1238, 165.13829),
                ball_vel: Vector3::new(391.0601, 152.0587, 344.67285),
                car_loc: Vector3::new(2699.6733, -4897.0317, 15.455973),
                car_rot: Rotation3::from_unreal_angles(0.002876214, 1.2708073, -0.0089162635),
                car_vel: Vector3::new(338.24823, 904.18585, 11.588186),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn hard_mode_pickup() {
        let test = TestRunner::start(
            CarryFromBounce::new(),
            TestScenario {
                ball_loc: Vector3::new(2087.2695, -3514.1238, 165.13829),
                ball_vel: Vector3::new(591.0601, 152.0587, 144.67285),
                car_loc: Vector3::new(2699.6733, -4897.0317, 15.455973),
                car_rot: Rotation3::from_unreal_angles(0.002876214, 1.2708073, -0.0089162635),
                car_vel: Vector3::new(338.24823, 904.18585, 11.588186),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn wait_for_side_wall_rebound() {
        let test = TestRunner::start(
            CarryFromBounce::new(),
            TestScenario {
                ball_loc: Vector3::new(-2591.3115, -1322.5367, 459.51404),
                ball_vel: Vector3::new(-930.0902, -258.3213, -359.45007),
                car_loc: Vector3::new(-3092.0574, -3168.787, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -3.0723717, 0.0),
                car_vel: Vector3::new(-1631.8348, -197.60046, 8.34),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }
}
