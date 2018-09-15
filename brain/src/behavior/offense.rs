use behavior::shoot::Shoot;
use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::simple_steer_towards;
use mechanics::HesitantDriveToLoc;
use nalgebra::Vector2;
use predict::estimate_intercept_car_ball_2;
use rlbot;
use simulate::rl;
use utils::{enemy_goal_center, one_v_one, ExtendPhysics, ExtendVector3};

pub struct Offense {
    min_distance: Option<f32>,
}

impl Offense {
    pub fn new() -> Offense {
        Offense { min_distance: None }
    }
}

impl Behavior for Offense {
    fn name(&self) -> &'static str {
        stringify!(Offense)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if !GetToFlatGround::on_flat_ground(packet) {
            return Action::call(GetToFlatGround::new());
        }

        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |t, &loc, vel| {
            Shoot::good_angle(loc, me.Physics.loc())
        });

        if Shoot::good_angle(intercept.ball_loc, me.Physics.loc()) {
            eeg.log(format!("Good angle found {:?}", intercept.ball_loc));
            return Action::call(Shoot::new());
        }

        // TODO: if angle is almost good, slightly adjust path such that good_angle
        // becomes true

        // TODO: otherwise drive to a point where me.y < ball.y, then slam the ball
        // sideways

        // also for the above, do something sane about possession e.g. if we clearly do
        // not have possession, probably just run back to defense for now?

        // For now, just fall back to a stupid behavior
        // TODO: possession! if have it, can wait. otherwise, 50/50
        let ball = intercept.ball_loc.to_2d();
        let goal = enemy_goal_center();
        let target_loc = ball + (ball - goal).normalize() * 1000.0;
        let target_loc = Vector2::new(
            target_loc
                .x
                .max(-rl::FIELD_MAX_X * 0.9)
                .min(rl::FIELD_MAX_X * 0.9),
            target_loc
                .y
                .max(-rl::FIELD_MAX_Y * 0.9)
                .min(rl::FIELD_MAX_Y * 0.9),
        );
        Action::call(HesitantDriveToLoc::new(target_loc))
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::offense::Offense;
    use behavior::root::RootBehavior;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use utils::ExtendPhysics;

    #[test]
    #[ignore] // TODO
    fn wait_for_curl_around_lip_near_post() {
        let test = TestRunner::start(
            Offense::new(),
            TestScenario {
                ball_loc: Vector3::new(2972.6848, 1018.38824, 101.33544),
                ball_vel: Vector3::new(-1029.0707, 2168.4673, -61.355755),
                car_loc: Vector3::new(3147.7668, 686.3356, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.689584, 0.0),
                car_vel: Vector3::new(-685.4966, 1531.4093, 83.56691),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn in_corner_barely_cant_reach() {
        let test = TestRunner::start(
            Offense::new(),
            TestScenario {
                ball_loc: Vector3::new(2230.9802, 2748.329, 93.14),
                ball_vel: Vector3::new(640.13696, 820.1949, 0.0),
                car_loc: Vector3::new(2847.7441, 1709.6339, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.4079068, 0.0000958738),
                car_vel: Vector3::new(286.62524, 1500.3096, 8.17),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn wait_for_ball_to_fall() {
        let test = TestRunner::start(
            RootBehavior::new(),
            TestScenario {
                ball_loc: Vector3::new(-3987.7068, -2086.639, 329.19128),
                ball_vel: Vector3::new(277.659, -238.58536, 992.14404),
                car_loc: Vector3::new(-2913.0967, -3791.6895, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.9806569, -0.0000958738),
                car_vel: Vector3::new(-352.9971, 833.215, 8.34),
                ..Default::default()
            },
        );

        test.sleep_millis(3000);
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().y >= 1000.0);
    }

    #[test]
    #[ignore] // TODO
    fn shoot_across_the_goal() {
        let test = TestRunner::start(
            RootBehavior::new(),
            TestScenario {
                ball_loc: Vector3::new(-1438.1412, 4697.7017, 92.71),
                ball_vel: Vector3::new(452.51102, -191.12935, 0.0),
                car_loc: Vector3::new(-2771.101, 4448.831, 17.0),
                car_rot: Rotation3::from_unreal_angles(-0.009491506, -0.038637143, -0.0000958738),
                car_vel: Vector3::new(974.10455, -28.867546, 8.429999),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(3000);
        assert!(test.has_scored());
    }
}
