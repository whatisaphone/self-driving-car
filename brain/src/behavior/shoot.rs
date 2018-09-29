use behavior::{aerial_shot::AerialShot, Action, Behavior};
use maneuvers::{GroundShot, JumpShot};
use nalgebra::Vector3;
use predict::estimate_intercept_car_ball;
use simulate::rl;
use strategy::Context;
use utils::ExtendPhysics;

pub struct Shoot;

impl Shoot {
    pub fn new() -> Shoot {
        Shoot
    }

    pub fn good_angle(ball_loc: Vector3<f32>, car_loc: Vector3<f32>) -> bool {
        // Aerials are not ready for prime-time yet
        if ball_loc.z >= JumpShot::MAX_BALL_Z {
            return false;
        }

        // This is woefully incomplete
        if ball_loc.x.abs() >= rl::FIELD_MAX_X || ball_loc.y.abs() >= rl::FIELD_MAX_Y {
            return false; // Ball is outside the field; clearly prediction has gone wrong somehow.
        } else if ball_loc.y.abs() >= rl::FIELD_MAX_Y - 250.0 && ball_loc.x.abs() >= rl::GOALPOST_X
        {
            return false;
        }

        GroundShot::good_angle(ball_loc, car_loc)
    }
}

impl Behavior for Shoot {
    fn name(&self) -> &'static str {
        stringify!(Shoot)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();

        let intercept = estimate_intercept_car_ball(ctx, me, |_t, &loc, _vel| {
            Self::good_angle(loc, me.Physics.loc())
        });

        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[Shoot] no good intercept");
            return Action::Abort;
        });

        if intercept.ball_loc.z < GroundShot::MAX_BALL_Z {
            return Action::call(GroundShot::new());
        }

        if intercept.ball_loc.z < JumpShot::MAX_BALL_Z {
            return Action::call(JumpShot::new());
        }

        Action::call(AerialShot::new())
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::shoot::Shoot;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::Runner2;

    #[test]
    fn awkwardly_angled_breakaway() {
        let test = TestRunner::start0(
            TestScenario::from_collect_row("570.0018	61.44401	-2595.4697	94.76237	0.4878059	0.18810439	-2.468271	-345.76355	848.0587	-6.6958303	-5.441	-2.2591	1.1357	1352.9867	-4963.935	18.86079	-0.02751578	0.14409833	0.07267234	590.5667	116.87245	5.3080044	0.17997983	0.10499983	2.2173882	1081.7532	-5051.4766	37.36723	0.628261	-1.6557405	-0.10153035	-12.958379	-17.438751	21.12848	-0.21157923	-0.12765418	-0.0040784255"),
//            TestScenario::from_collect_row("	-0.02751578	0.14409833	0.07267234	590.5667	116.87245	5.3080044	0.17997983	0.10499983	2.2173882	1081.7532	-5051.4766	37.36723	0.628261	-1.6557405	-0.10153035	-12.958379	-17.438751	21.12848	-0.21157923	-0.12765418	-0.0040784255"),
//            TestScenario {
//                ball_loc: Vector3::new(60.0, -2500.0, 90.0),
//                ball_vel: Vector3::new(-350.0, 900.0, 0.0),
//                car_loc: Vector3::new(1500.0, -5000.0, 0.0),
//                car_vel: Vector3::new(0.0, 0.0, 0.0),
//                ..Default::default()
//            }
        );
        test.set_behavior(Shoot::new());
        test.sleep_millis(7000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn awkward_breakaway_2() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(2944.1208, -2035.1736, 309.80853),
            ball_vel: Vector3::new(-457.17484, 450.0548, -17.921162),
            car_loc: Vector3::new(3666.633, -3609.852, 16.672583),
            car_rot: Rotation3::from_unreal_angles(-0.0055606803, 2.2166023, -0.004697816),
            car_vel: Vector3::new(-805.1952, 1035.4634, 14.579811),
            ..Default::default()
        });
        test.set_behavior(Shoot::new());
        test.sleep_millis(7000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn awkward_bouncing_breakaway() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-3083.084, -3752.2727, 567.4716),
            ball_vel: Vector3::new(738.0995, 1098.1213, 425.59665),
            car_loc: Vector3::new(-2996.7085, -3469.6912, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.009683254, 2.7731497, 0.0000958738),
            car_vel: Vector3::new(-917.587, 322.66766, 8.34),
            ..Default::default()
        });
        test.set_behavior(Shoot::new());
        test.sleep_millis(6000);
        assert!(test.has_scored());
    }

    #[test]
    fn fast_falling_ball() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(3862.044, 1163.3925, 1456.4243),
            ball_vel: Vector3::new(2532.4116, 897.6915, 396.8566),
            car_loc: Vector3::new(1530.9783, 45.435856, 16.924282),
            car_rot: Rotation3::from_unreal_angles(-0.010162623, 0.8, -0.0006711166),
            car_vel: Vector3::new(1301.4751, 366.96378, 9.762962),
            ..Default::default()
        });

        // Temp fix until ball prediction can handle walls. Also perhaps see source
        // control to restore the previous car rotation.
        test.sleep_millis(50);

        test.set_behavior(Shoot::new());

        test.sleep_millis(4000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn speedy_angle_adjust() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(856.57855, 2725.1563, 93.15),
            ball_vel: Vector3::new(381.07864, -367.86865, 0.0),
            car_loc: Vector3::new(2762.3386, 1111.3347, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -2.765384, 0.0),
            car_vel: Vector3::new(-1621.1874, -744.78345, 8.34),
            ..Default::default()
        });
        test.set_behavior(Shoot::new());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn close_fast_rolling_redirect() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(99.87652, 1684.7692, 93.14),
            ball_vel: Vector3::new(941.7007, 1168.9557, 0.0),
            car_loc: Vector3::new(1592.5865, 3.3359032, 17.0),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 0.77255106, 0.0000958738),
            car_vel: Vector3::new(1499.8735, 1302.2483, 8.44),
            ..Default::default()
        });
        test.set_behavior(Shoot::new());
        test.sleep_millis(3000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn far_rolling_along_side_wall() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(3827.3035, -2371.3047, 103.192085),
            ball_vel: Vector3::new(56.775806, 750.3081, -420.45816),
            car_loc: Vector3::new(3082.6775, -4116.359, 16.99),
            car_rot: Rotation3::from_unreal_angles(-0.009395632, 1.3753097, -0.0000958738),
            car_vel: Vector3::new(216.59224, 1339.4177, 8.41),
            ..Default::default()
        });
        test.set_behavior(Shoot::new());
        test.sleep_millis(6000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn awkward_corner_angle() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(3074.1807, 4219.743, 506.9326),
            ball_vel: Vector3::new(-1596.3938, 1474.6923, -355.48773),
            car_loc: Vector3::new(-970.7269, 2484.3645, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.5245851, -0.0000958738),
            car_vel: Vector3::new(64.24027, 1407.491, 8.309999),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(4000);
        assert!(test.has_scored());
    }
}
