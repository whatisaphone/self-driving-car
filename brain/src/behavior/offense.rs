use behavior::{shoot::Shoot, Action, Behavior};
use eeg::EEG;
use maneuvers::{BounceShot, GetToFlatGround};
use plan::hit_angle::{feasible_hit_angle_away, feasible_hit_angle_toward};
use predict::{estimate_intercept_car_ball_2, Intercept};
use rlbot;
use std::f32::consts::PI;
use utils::{
    enemy_goal_center, my_car, my_goal_center_2d, one_v_one, ExtendPhysics, ExtendVector3,
    WallRayCalculator,
};

pub struct Offense;

impl Offense {
    pub fn new() -> Self {
        Offense
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
        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |_t, &loc, _vel| {
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
        tepid_hit(packet, eeg, intercept)
    }
}

fn tepid_hit(packet: &rlbot::LiveDataPacket, eeg: &mut EEG, intercept: Intercept) -> Action {
    eeg.log("no good hit; going for a tepid hit");

    let me = my_car(packet);
    let ball = intercept.ball_loc.to_2d();
    let goal = enemy_goal_center();
    let avoid = my_goal_center_2d();

    // If we're retreating, explicitly hit it away from our goal to avoid an own
    // goal.
    let naive_wall = WallRayCalculator::calc_segment(me.Physics.loc().to_2d(), ball);
    let theta = if (naive_wall - goal).norm() < (naive_wall - avoid).norm() {
        eeg.log("hitting toward enemy goal");
        feasible_hit_angle_toward(ball, me.Physics.loc().to_2d(), goal, PI / 6.0)
    } else {
        eeg.log("hitting away from own goal");
        feasible_hit_angle_away(ball, me.Physics.loc().to_2d(), avoid, PI / 6.0)
    };
    let aim_loc = WallRayCalculator::calc_ray(ball, theta);
    Action::call(BounceShot::new().with_target_loc(aim_loc))
}

#[cfg(test)]
mod integration_tests {
    use behavior::{offense::Offense, root::RootBehavior};
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::Runner2;
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

    #[test]
    #[ignore] // TODO
    fn awkward_breakaway_shot() {
        let test = TestRunner::start(
            RootBehavior::new(),
            TestScenario {
                ball_loc: Vector3::new(1463.9786, -1842.5327, 93.15),
                ball_vel: Vector3::new(-1061.1782, 224.63322, 0.0),
                car_loc: Vector3::new(2019.6799, -3689.1953, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 2.376328, 0.0),
                car_vel: Vector3::new(-962.35034, 924.71826, 8.309999),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(5000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn juicy_bouncing() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(3811.8657, 1580.2241, 1172.8545),
            ball_vel: Vector3::new(-1703.4757, 753.66534, 210.33441),
            car_loc: Vector3::new(-1648.8497, 1804.7543, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.009779127, 1.909902, 0.0000958738),
            car_vel: Vector3::new(-702.66034, 1446.7336, 8.615206),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());

        test.sleep_millis(3000);
        assert!(test.has_scored());
    }
}
