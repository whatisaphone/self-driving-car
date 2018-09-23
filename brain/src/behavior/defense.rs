use behavior::{Action, Behavior, Chain, Priority};
use eeg::{color, Drawable, EEG};
use maneuvers::{BounceShot, GroundShot, JumpShot, PanicDefense};
use nalgebra::{Rotation2, Vector2};
use predict::{
    estimate_intercept_car_ball, estimate_intercept_car_ball_3, is_sane_ball_loc, Intercept,
};
use rlbot;
use std::f32::consts::PI;
use strategy::{Context, Scenario};
use utils::{my_car, my_goal_center_2d, ExtendPhysics, ExtendVector3, Wall, WallRayCalculator};

pub struct Defense {
    finished: bool,
}

impl Defense {
    pub fn new() -> Defense {
        Defense { finished: false }
    }
}

impl Behavior for Defense {
    fn name(&self) -> &'static str {
        stringify!(Defense)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if self.finished {
            return Action::Return;
        }

        self.finished = true;

        let me = my_car(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        if !is_sane_ball_loc(intercept.ball_loc) {
            eeg.draw(Drawable::print("panicking", color::GREEN));
            return Action::call(PanicDefense::new());
        }

        Action::call(Chain::new(
            Priority::Save,
            vec![
                Box::new(PushToOwnCorner::new()),
                Box::new(PanicDefense::new().use_boost(false)),
            ],
        ))
    }
}

struct PushToOwnCorner;

impl PushToOwnCorner {
    fn new() -> Self {
        PushToOwnCorner
    }
}

impl Behavior for PushToOwnCorner {
    fn name(&self) -> &'static str {
        stringify!(PushToOwnCorner)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let ball_trajectory = WallRayCalculator::calculate(
            ctx.packet.GameBall.Physics.loc().to_2d(),
            ctx.packet.GameBall.Physics.loc().to_2d() + ctx.packet.GameBall.Physics.vel().to_2d(),
        );
        let already_cornering = match WallRayCalculator::wall_for_point(ball_trajectory) {
            Wall::OwnGoal => false,
            _ => true,
        };

        let me_intercept =
            estimate_intercept_car_ball_3(ctx.enemy(), &ctx.packet.GameBall, |_t, &loc, _vel| {
                loc.z < JumpShot::MAX_BALL_Z
            });

        let enemy_shootable_intercept =
            estimate_intercept_car_ball_3(ctx.enemy(), &ctx.packet.GameBall, |_t, &loc, _vel| {
                loc.z < JumpShot::MAX_BALL_Z
                    && GroundShot::good_angle(ctx.enemy().Physics.loc(), loc)
            });

        assert_eq!(ctx.me().Team, 0); // or the colors below won't be right
        if let Some(ref i) = me_intercept {
            ctx.eeg.draw(Drawable::GhostBall2(i.ball_loc, color::BLUE));
        }
        if let Some(ref i) = enemy_shootable_intercept {
            ctx.eeg
                .draw(Drawable::GhostBall2(i.ball_loc, color::ORANGE));
        }

        match (me_intercept, enemy_shootable_intercept) {
            (_, None) => {
                if already_cornering {
                    ctx.eeg.log("Safe for now");
                    Action::Return
                } else {
                    ctx.eeg.log("Hitting away from goal");
                    Action::call(HitToOwnCorner::new())
                }
            }
            (None, _) => {
                ctx.eeg.log("Can't reach ball; panicking");
                Action::call(PanicDefense::new())
            }
            (Some(me), Some(enemy)) => {
                if me.time < enemy.time - Scenario::POSSESSION_CONTESTABLE {
                    ctx.eeg.log("Swatting ball away from enemy");
                    Action::call(HitToOwnCorner::new())
                } else if me.time < enemy.time + Scenario::POSSESSION_CONTESTABLE {
                    ctx.eeg.log("Defensive race");
                    Action::call(HitToOwnCorner::new())
                } else {
                    ctx.eeg.log("Can't reach ball before enemy; panicking");
                    Action::call(PanicDefense::new())
                }
            }
        }
    }
}

pub struct HitToOwnCorner;

impl HitToOwnCorner {
    pub fn new() -> Self {
        HitToOwnCorner
    }
}

impl Behavior for HitToOwnCorner {
    fn name(&self) -> &'static str {
        stringify!(HitToOwnCorner)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg.log("redirect to own corner");

        let intercept =
            estimate_intercept_car_ball_3(ctx.me(), &ctx.packet.GameBall, |_t, &loc, _vel| {
                loc.z < BounceShot::MAX_BALL_Z
            });

        match intercept {
            None => Action::Return,
            Some(intercept) => {
                let target_loc = Self::aim_loc(ctx, &intercept);
                Action::call(BounceShot::new().with_target_loc(target_loc))
            }
        }
    }
}

impl HitToOwnCorner {
    fn aim_loc(ctx: &mut Context, intercept: &Intercept) -> Vector2<f32> {
        let avoid = my_goal_center_2d();

        let me = my_car(ctx.packet);
        let me_loc = me.Physics.loc().to_2d();
        let ball_loc = intercept.ball_loc.to_2d();
        let me_to_ball = ball_loc - me_loc;

        let ltr_dir = Rotation2::new(PI / 6.0) * me_to_ball;
        let ltr = WallRayCalculator::calculate(ball_loc, ball_loc + ltr_dir);
        let rtl_dir = Rotation2::new(-PI / 6.0) * me_to_ball;
        let rtl = WallRayCalculator::calculate(ball_loc, ball_loc + rtl_dir);

        if (avoid - ltr.coords.to_2d()).norm() > (avoid - rtl.coords.to_2d()).norm() {
            ctx.eeg.log("push from left to right");
            ltr.coords.to_2d()
        } else {
            ctx.eeg.log("push from right to left");
            rtl.coords.to_2d()
        }
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::{defense::Defense, runner::PUSHED, HitToOwnCorner};
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::Runner2;
    use utils::ExtendPhysics;

    #[test]
    fn bouncing_save() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-3143.9788, -241.96017, 1023.1816),
            ball_vel: Vector3::new(717.56323, -1200.3536, 331.91443),
            car_loc: Vector3::new(-4009.9998, -465.8022, 86.914),
            car_rot: Rotation3::from_unreal_angles(-0.629795, -0.7865487, 0.5246214),
            car_vel: Vector3::new(982.8443, -1059.1908, -935.80194),
            ..Default::default()
        });

        let start_time = test.sniff_packet().GameInfo.TimeSeconds;
        test.set_behavior(Runner2::new());

        let mut max_z = 0.0_f32;
        loop {
            let packet = test.sniff_packet();
            let elapsed = packet.GameInfo.TimeSeconds - start_time;
            if elapsed >= 4.0 {
                break;
            }
            if elapsed >= 1.0 && packet.GameBall.Physics.Velocity.Z > 0.0 {
                max_z = max_z.max(packet.GameBall.Physics.Location.Z);
            }
        }

        test.examine_eeg(|eeg| {
            assert!(PUSHED == ">");
            assert!(eeg.log.iter().any(|x| x == "baseline: Defense"));
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(!eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(eeg.log.iter().any(|x| x == "push from left to right"));
        });

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.Location);
        assert!(packet.GameBall.Physics.Location.X >= 800.0);
        assert!(packet.GameBall.Physics.Location.Y >= -4000.0);

        // Should power-shot, meaning the ball bounces high.
        assert!(max_z >= 500.0, "{}", max_z);
    }

    #[test]
    fn redirect_away_from_goal() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(-2667.985, 779.3049, 186.92154),
                ball_vel: Vector3::new(760.02606, -1394.5569, -368.39642),
                car_loc: Vector3::new(-2920.1282, 1346.1251, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.1758921, 0.0),
                car_vel: Vector3::new(688.0767, -1651.0865, 8.181303),
                ..Default::default()
            },
        );

        // This result is just *okay*
        test.sleep_millis(100);
        test.examine_eeg(|eeg| {
            assert!(PUSHED == ">");
            assert!(eeg.log.iter().any(|x| x == "baseline: Defense"));
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(!eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(eeg.log.iter().any(|x| x == "push from left to right"));
        });
    }

    #[test]
    #[ignore] // TODO
    fn last_second_save() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(-1150.811, -1606.0569, 102.36157),
                ball_vel: Vector3::new(484.87906, -1624.8169, 32.10115),
                car_loc: Vector3::new(-1596.7955, -1039.2034, 17.0),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.4007162, 0.0000958738),
                car_vel: Vector3::new(242.38637, -1733.6719, 8.41),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(3000);
        assert!(!test.enemy_has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn slow_bouncer() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(-2849.355, -2856.8281, 1293.4608),
                ball_vel: Vector3::new(907.1093, -600.48956, 267.59674),
                car_loc: Vector3::new(1012.88916, -3626.2666, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -0.8467574, 0.0),
                car_vel: Vector3::new(131.446, -188.83897, 8.33),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(!test.enemy_has_scored());
    }

    #[test]
    fn falling_save_from_the_side() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(2353.9868, -5024.7144, 236.38712),
                ball_vel: Vector3::new(-1114.3461, 32.5409, 897.3589),
                car_loc: Vector3::new(2907.8083, -4751.0806, 17.010809),
                car_rot: Rotation3::from_unreal_angles(-0.018216021, -2.7451544, -0.0073822825),
                car_vel: Vector3::new(-1412.7858, -672.18933, -6.2963967),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(3000);

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.vel());
        assert!(packet.GameBall.Physics.vel().x < -2000.0);
    }

    #[test]
    fn retreating_push_to_corner() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(436.92395, 1428.1085, 93.15),
            ball_vel: Vector3::new(-112.55582, -978.27814, 0.0),
            car_loc: Vector3::new(1105.1365, 2072.0022, 17.0),
            car_rot: Rotation3::from_unreal_angles(-0.009491506, -2.061095, -0.0000958738),
            car_vel: Vector3::new(-546.6459, -1095.6816, 8.29),
            ..Default::default()
        });

        test.set_behavior(Defense::new());

        test.sleep_millis(1000);

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(!eeg.log.iter().any(|x| x == "push from left to right"));
            assert!(eeg.log.iter().any(|x| x == "push from right to left"));
        });

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.Velocity);
        assert!(packet.GameBall.Physics.vel().norm() >= 2000.0);
    }

    #[test]
    #[ignore] // TODO
    fn retreating_push_to_corner_from_awkward_side() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(1948.3385, 1729.5826, 97.89405),
            ball_vel: Vector3::new(185.58005, -1414.3043, -5.051092),
            car_loc: Vector3::new(896.22095, 1962.7969, 15.68419),
            car_rot: Rotation3::from_unreal_angles(-0.0131347105, -2.0592732, -0.010450244),
            car_vel: Vector3::new(-660.1856, -1449.2916, -3.7354965),
            ..Default::default()
        });

        test.set_behavior(Defense::new());

        test.sleep_millis(2000);

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(!eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(eeg.log.iter().any(|x| x == "push from left to right"));
        });

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.Velocity);
        assert!(packet.GameBall.Physics.vel().norm() >= 2000.0);
    }

    #[test]
    #[ignore] // TODO
    fn retreating_push_to_corner_from_awkward_angle() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-2365.654, -86.64402, 114.0818),
            ball_vel: Vector3::new(988.47064, -1082.8477, -115.50357),
            car_loc: Vector3::new(-2708.0007, -17.896847, 250.98781),
            car_rot: Rotation3::from_unreal_angles(0.28522456, -0.8319928, -0.05263472),
            car_vel: Vector3::new(550.82794, -1164.1539, 277.63806),
            ..Default::default()
        });

        test.set_behavior(Defense::new());

        test.sleep_millis(2000);

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(!eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(eeg.log.iter().any(|x| x == "push from left to right"));
        });

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.Velocity);
        assert!(packet.GameBall.Physics.vel().norm() >= 2000.0);
    }

    #[test]
    fn push_from_corner_to_corner() {
        let test = TestRunner::start(
            HitToOwnCorner::new(),
            TestScenario {
                ball_loc: Vector3::new(1620.9868, -4204.8145, 93.14),
                ball_vel: Vector3::new(-105.58675, 298.33023, 0.0),
                car_loc: Vector3::new(3361.587, -4268.589, 16.258373),
                car_rot: Rotation3::from_unreal_angles(-0.0066152923, 1.5453898, -0.005752428),
                car_vel: Vector3::new(89.86856, 1188.811, 7.4339933),
                ..Default::default()
            },
        );

        test.sleep_millis(2000);
        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(!eeg.log.iter().any(|x| x == "push from left to right"));
        });
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().norm() >= 2000.0);
    }

    #[test]
    #[ignore] // TODO
    fn push_from_corner_to_corner_2() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(2517.809, -4768.475, 93.13),
                ball_vel: Vector3::new(-318.6226, 490.17892, 0.0),
                car_loc: Vector3::new(3742.2703, -3277.4558, 16.954643),
                car_rot: Rotation3::from_unreal_angles(-0.009108011, 2.528288, -0.0015339808),
                car_vel: Vector3::new(-462.4023, 288.65112, 9.278907),
                boost: 10,
                ..Default::default()
            },
        );

        test.sleep_millis(2000);
        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(!eeg.log.iter().any(|x| x == "push from left to right"));
        });
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().norm() >= 2000.0);
    }

    #[test]
    fn same_side_corner_push() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(-2545.9438, -4174.64, 318.26862),
                ball_vel: Vector3::new(985.6374, -479.52872, -236.39767),
                car_loc: Vector3::new(-1808.3466, -3266.7039, 16.41444),
                car_rot: Rotation3::from_unreal_angles(-0.009203885, -0.65855706, -0.0015339808),
                car_vel: Vector3::new(947.339, -565.98175, 15.669456),
                ..Default::default()
            },
        );

        test.sleep_millis(2000);
        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(eeg.log.iter().any(|x| x == "push from right to left"));
            assert!(!eeg.log.iter().any(|x| x == "push from left to right"));
        });
        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.vel());
        assert!(packet.GameBall.Physics.vel().x < -500.0);
    }

    #[test]
    #[ignore] // TODO
    fn slow_rolling_save() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(1455.9731, -4179.0796, 93.15),
                ball_vel: Vector3::new(-474.48724, -247.0518, 0.0),
                car_loc: Vector3::new(2522.638, -708.08484, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 2.6835077, 0.0),
                car_vel: Vector3::new(-1433.151, 800.56586, 8.33),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(5000);
        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().x < -1000.0);
    }

    #[test]
    fn slow_retreating_save() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(1446.3031, -2056.4917, 213.57251),
                ball_vel: Vector3::new(-1024.0333, -1593.1566, -244.15135),
                car_loc: Vector3::new(314.3022, -1980.4884, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.7653242, 0.0),
                car_vel: Vector3::new(-268.87683, -1383.9724, 8.309999),
                ..Default::default()
            },
        );

        test.sleep_millis(2000);
        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.loc().x >= 1000.0);
        assert!(packet.GameBall.Physics.vel().x >= 500.0);
    }

    #[test]
    fn fast_retreating_save() {
        let test = TestRunner::start(
            Runner2::new(),
            TestScenario {
                ball_loc: Vector3::new(63.619453, -336.2556, 93.03),
                ball_vel: Vector3::new(-189.17311, -1918.067, 0.0),
                car_loc: Vector3::new(-103.64991, 955.411, 16.99),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.5927514, 0.0),
                car_vel: Vector3::new(-57.26778, -2296.9263, 8.53),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);
        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.loc().x < 1000.0);
        assert!(packet.GameBall.Physics.vel().x < 500.0);
    }
}
