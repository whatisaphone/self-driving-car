use crate::{
    behavior::{
        higher_order::Chain,
        strike::{
            BounceShot, GroundedHit, GroundedHitAimContext, GroundedHitTarget,
            GroundedHitTargetAdjust,
        },
    },
    helpers::intercept::{naive_ground_intercept_2, NaiveIntercept},
    routing::{behavior::FollowRoute, plan::GroundIntercept},
    strategy::{Action, Behavior, Context, Game, Priority, Scenario},
    utils::geometry::Line2,
};
use common::{prelude::*, Speed};
use nalgebra::{Point2, Point3};
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;

pub struct Shoot;

impl Shoot {
    pub fn new() -> Self {
        Self
    }

    pub fn viable_shot(
        game: &Game<'_>,
        car_loc: Point3<f32>,
        ball_loc: Point3<f32>,
    ) -> Option<Shot> {
        // Aerials are not ready for prime-time yet
        if ball_loc.z >= GroundedHitTarget::MAX_BALL_Z {
            return None;
        }

        // Get a "naive" location for initial checks, directly across from the car.
        let goal = game.enemy_goal();
        let goalline = Line2::from_origin_dir(goal.center_2d, goal.normal_2d.ortho().to_axis());
        let aim_loc = goalline
            .intersect(Line2::from_points(car_loc.to_2d(), ball_loc.to_2d()))
            .unwrap_or(goal.center_2d);
        let aim_loc = goal.closest_point(aim_loc);
        // Fix weirdness when the ball is halfway in the goal. If the aim location is
        // directly on the surface of the goal plane, it's in "front" of the ball, and
        // since we can't hit the ball backwards (naturally), we would abort. Prevent
        // that weirdness and take the easy goal.
        let aim_loc = aim_loc + (aim_loc - car_loc.to_2d()).normalize() * 500.0;

        let ball_to_goal = aim_loc - ball_loc.to_2d();
        let car_to_ball = ball_loc.to_2d() - car_loc.to_2d();
        if car_to_ball.angle_to(&ball_to_goal).abs() >= PI / 6.0 {
            return None;
        }

        let goal_angle = (ball_loc.to_2d() - aim_loc)
            .to_axis()
            .angle_to(&goal.normal_2d);
        if goal_angle.abs() >= PI * (5.0 / 12.0) {
            return None;
        }

        // If all preconditions succeeded, return the actual spot we should aim at. This
        // will probably end up a lot closer to the center of the goal.
        let aim_loc = BounceShot::aim_loc(goal, car_loc.to_2d(), ball_loc.to_2d());

        Some(Shot { aim_loc })
    }

    fn aim(ctx: &mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> {
        match Self::aim_calc(ctx.game, ctx.scenario, ctx.car) {
            Some(i) => Ok(GroundedHitTarget::new(
                i.time,
                GroundedHitTargetAdjust::RoughAim,
                i.data.aim_loc,
            )
            .jump(!Self::is_chippable(ctx, &i))),
            None => Err(()),
        }
    }

    pub fn is_chippable(
        ctx: &mut GroundedHitAimContext<'_, '_>,
        intercept: &NaiveIntercept<Shot>,
    ) -> bool {
        // `ctx.intercept_ball_loc` is the soonest possible intercept.
        // `intercept` is the possibly-later "strategic" intercept that we'll wait for
        // before shooting.

        let ball_loc = intercept.ball_loc.to_2d();
        let ball_vel = intercept.ball_vel.to_2d();
        let car_loc = intercept.car_loc.to_2d();
        let car_forward_axis = (ball_loc - car_loc).to_axis();
        let car_speed = intercept.car_speed;
        let car_vel = (ball_loc - ctx.car.Physics.loc_2d()).normalize() * car_speed;

        let shot_distance = (intercept.data.aim_loc - ball_loc).norm();
        let car_speed_towards_ball = (car_vel - ball_vel).dot(&car_vel.to_axis());
        let approach_angle = car_forward_axis
            .angle_to(&(intercept.data.aim_loc - ball_loc))
            .abs();
        let shot_angle = (ball_loc - car_loc)
            .angle_to(&(intercept.data.aim_loc - ball_loc))
            .abs();

        // A very poor estimate of the portion of the shot's trajectory (a parabola)
        // that will be above the crossbar (meaning we would miss the shot).
        let min_distance_cutoff =
            linear_interpolate(&[2000.0, 3000.0], &[2000.0, 750.0], car_speed_towards_ball);
        let max_distance_cutoff = linear_interpolate(
            &[1500.0, 2000.0, 2500.0],
            &[1800.0, 2800.0, 5000.0],
            car_speed_towards_ball,
        );

        // Check if the enemy is directly in the way of the shot.
        let mut enemy_blocking_dist = 9999.0;
        let mut enemy_blocking_ortho_dist = 9999.0;
        let mut enemy_blocking_angle = PI;
        if let Some(enemy) = ctx.scenario.primary_enemy() {
            let ball_loc = intercept.ball_loc.to_2d();
            let ball_to_goal = (intercept.data.aim_loc - ball_loc).to_axis();
            let ball_to_enemy = enemy.Physics.loc_2d() - ball_loc;
            enemy_blocking_dist = ball_to_enemy.dot(&ball_to_goal);
            enemy_blocking_ortho_dist = ball_to_enemy.dot(&ball_to_goal.ortho()).abs();
            enemy_blocking_angle = ball_to_goal.angle_to(&ball_to_enemy).abs();
        };
        // The enemy is blocking if they are far enough away that this isn't a 50/50,
        // and close to the line from the ball to the goal.
        let enemy_blocking = enemy_blocking_dist >= 750.0 && enemy_blocking_ortho_dist < 1000.0;

        let min_car_speed = linear_interpolate(
            &[2500.0, 5000.0],
            &[1000.0, if enemy_blocking { 1000.0 } else { 2000.0 }],
            shot_distance,
        );

        ctx.eeg.print_value("car_speed", Speed(car_speed));
        ctx.eeg.print_value("min_car_speed", Speed(min_car_speed));
        ctx.eeg
            .print_value("rel_speed", Speed(car_speed_towards_ball));
        ctx.eeg.print_angle("approach_angle", approach_angle);
        ctx.eeg.print_angle("shot_angle", shot_angle);
        ctx.eeg.print_distance("shot_distance", shot_distance);
        ctx.eeg
            .print_distance("min_distance_cutoff", min_distance_cutoff);
        ctx.eeg
            .print_distance("max_distance_cutoff", max_distance_cutoff);
        ctx.eeg
            .print_angle("enemy_blocking_angle", enemy_blocking_angle);
        ctx.eeg.print_distance("blocking", enemy_blocking_dist);
        ctx.eeg
            .print_distance("blocking ortho", enemy_blocking_ortho_dist);

        // If the ball is rolling towards us, take the easy chip and hopefully get it
        // over the opponent's head.
        car_speed >= min_car_speed
            && car_speed_towards_ball * 0.85 >= car_speed
            && (shot_distance < min_distance_cutoff || shot_distance >= max_distance_cutoff)
            && ctx.intercept_ball_loc.z < 110.0
            && ctx.intercept_ball_vel.z.abs() < 200.0
            && approach_angle < PI / 6.0
            && shot_angle < PI / 6.0
    }

    fn aim_calc(
        game: &Game<'_>,
        scenario: &Scenario<'_>,
        car: &common::halfway_house::PlayerInfo,
    ) -> Option<NaiveIntercept<Shot>> {
        naive_ground_intercept_2(&car.into(), scenario.ball_prediction(), |ball| {
            Self::viable_shot(game, car.Physics.loc(), ball.loc)
        })
    }
}

pub struct Shot {
    aim_loc: Point2<f32>,
}

impl Behavior for Shoot {
    fn name(&self) -> &str {
        name_of_type!(Shoot)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let intercept = Self::aim_calc(ctx.game, &ctx.scenario, ctx.me());
        if intercept.is_none() {
            ctx.eeg.log(self.name(), "no viable shot");
            return Action::Abort;
        }

        Action::tail_call(Chain::new(Priority::Strike, vec![
            Box::new(FollowRoute::new(GroundIntercept::new()).same_ball_trajectory(true)),
            Box::new(GroundedHit::hit_towards(Self::aim)),
        ]))
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::offense::Shoot,
        integration_tests::{TestRunner, TestScenario},
    };
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    fn awkwardly_angled_breakaway() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(61.44401, -2595.4697, 94.76237),
                ball_vel: Vector3::new(-345.76355, 848.0587, -6.6958303),
                car_loc: Point3::new(1352.9867, -4963.935, 18.86079),
                car_rot: Rotation3::from_unreal_angles(-0.02751578, 0.14409834, 0.07267234),
                car_vel: Vector3::new(590.5667, 116.87245, 5.3080044),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(7000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "this usually works, but only by coincidence")]
    fn awkward_breakaway_2() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2944.1208, -2035.1736, 309.80853),
                ball_vel: Vector3::new(-457.17484, 450.0548, -17.921162),
                car_loc: Point3::new(3666.633, -3609.852, 16.672583),
                car_rot: Rotation3::from_unreal_angles(-0.0055606803, 2.2166023, -0.004697816),
                car_vel: Vector3::new(-805.1952, 1035.4634, 14.579811),
                ..Default::default()
            })
            .behavior(Shoot::new())
            .run_for_millis(6000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "I don't even")]
    fn awkward_bouncing_breakaway() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-3083.084, -3752.2727, 567.4716),
                ball_vel: Vector3::new(738.0995, 1098.1213, 425.59665),
                car_loc: Point3::new(-2996.7085, -3469.6912, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009683254, 2.7731497, 0.0000958738),
                car_vel: Vector3::new(-917.587, 322.66766, 8.34),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(7000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "The great bankruptcy of 2018")]
    fn fast_falling_ball() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(3862.044, 1163.3925, 1456.4243),
                ball_vel: Vector3::new(2532.4116, 897.6915, 396.8566),
                car_loc: Point3::new(1530.9783, 45.435856, 16.924282),
                car_rot: Rotation3::from_unreal_angles(-0.010162623, 0.8, -0.0006711166),
                car_vel: Vector3::new(1301.4751, 366.96378, 9.762962),
                ..Default::default()
            })
            .run();

        // Temp fix until ball prediction can handle walls. Also perhaps see source
        // control to restore the previous car rotation.
        test.sleep_millis(50);

        test.set_behavior(Shoot::new());

        test.sleep_millis(4000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "oops, this got broken at some point")]
    fn speedy_angle_adjust() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(856.57855, 2725.1563, 93.15),
                ball_vel: Vector3::new(381.07864, -367.86865, 0.0),
                car_loc: Point3::new(2762.3386, 1111.3347, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -2.765384, 0.0),
                car_vel: Vector3::new(-1621.1874, -744.78345, 8.34),
                ..Default::default()
            })
            .behavior(Shoot::new())
            .run_for_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    fn close_fast_rolling_redirect() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(99.87652, 1684.7692, 93.14),
                ball_vel: Vector3::new(941.7007, 1168.9557, 0.0),
                car_loc: Point3::new(1592.5865, 3.3359032, 17.0),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 0.77255106, 0.0000958738),
                car_vel: Vector3::new(1499.8735, 1302.2483, 8.44),
                ..Default::default()
            })
            .behavior(Shoot::new())
            .run_for_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "TODO")]
    fn far_rolling_along_side_wall() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(3827.3035, -2371.3047, 103.192085),
                ball_vel: Vector3::new(56.775806, 750.3081, -420.45816),
                car_loc: Point3::new(3082.6775, -4116.359, 16.99),
                car_rot: Rotation3::from_unreal_angles(-0.009395632, 1.3753097, -0.0000958738),
                car_vel: Vector3::new(216.59224, 1339.4177, 8.41),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(6000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "TODO")]
    fn awkward_corner_angle() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(3074.1807, 4219.743, 506.9326),
                ball_vel: Vector3::new(-1596.3938, 1474.6923, -355.48773),
                car_loc: Point3::new(-970.7269, 2484.3645, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.5245851, -0.0000958738),
                car_vel: Vector3::new(64.24027, 1407.491, 8.309999),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(4000);
        assert!(test.has_scored());
    }

    #[test]
    fn low_boost_shot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1847.84, 1641.52, 503.91998),
                ball_vel: Vector3::new(-796.701, 953.001, 192.491),
                car_loc: Point3::new(2106.05, 1107.9299, 176.58),
                car_rot: Rotation3::from_unreal_angles(0.6417668, 2.9078417, 0.24533166),
                car_vel: Vector3::new(-695.61096, 573.141, -247.531),
                ..Default::default()
            })
            .starting_boost(12.0)
            .soccar()
            .run_for_millis(3000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "works, but is flaky")]
    fn high_lobbed_shot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2966.91, 2058.1199, 1604.51),
                ball_vel: Vector3::new(-395.271, 175.991, -694.71094),
                car_loc: Point3::new(3739.39, 938.52997, 15.82),
                car_rot: Rotation3::from_unreal_angles(0.007055527, 2.3592541, -0.018111816),
                car_vel: Vector3::new(-938.84094, 909.071, 25.890999),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3500);

        assert!(test.has_scored());
    }

    #[test]
    fn another_forward_shot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2723.44, -441.38, 105.6),
                ball_vel: Vector3::new(161.55101, 822.9509, -354.241),
                car_loc: Point3::new(-2829.92, -1540.23, 16.5),
                car_rot: Rotation3::from_unreal_angles(-0.008225171, 1.4523904, -0.00015447255),
                car_vel: Vector3::new(75.461, 606.49097, 17.271),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(4000);

        assert!(test.has_scored());
    }

    #[test]
    fn chip_it_over_reliefbots_head() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1504.64, 2860.74, 98.34),
                ball_vel: Vector3::new(-310.841, -581.85095, -65.701),
                car_loc: Point3::new(-1992.88, 1130.71, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.0095118955, 0.8456476, -0.0000147181245),
                car_vel: Vector3::new(326.201, 386.471, 8.351),
                enemy_loc: Point3::new(-340.63, 5980.25, 250.68999),
                enemy_rot: Rotation3::from_unreal_angles(-0.7842756, -0.05550343, -1.5506147),
                enemy_vel: Vector3::new(692.891, -26.440998, -638.691),
                ..Default::default()
            })
            .starting_boost(80.0)
            .soccar()
            .run_for_millis(4000);

        assert!(test.has_scored());
    }

    #[test]
    fn chip_it_over_reliefbots_head_2() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2401.8298, 1984.36, 93.14),
                ball_vel: Vector3::new(-431.231, -335.451, 0.0),
                car_loc: Point3::new(-3290.5598, 1121.99, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009506902, 1.3701476, 0.0000138256455),
                car_vel: Vector3::new(133.77101, 602.08093, 8.351),
                enemy_loc: Point3::new(-227.01999, 4650.29, 86.189995),
                enemy_rot: Rotation3::from_unreal_angles(1.2419531, 0.794232, 3.0616176),
                enemy_vel: Vector3::new(-1023.441, -1180.4409, -14.081),
                ..Default::default()
            })
            .starting_boost(80.0)
            .soccar()
            .run_for_millis(4000);

        assert!(test.has_scored());
    }

    #[test]
    fn chip_it_over_reliefbots_head_3() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(141.26, 1363.0499, 93.15),
                ball_vel: Vector3::new(-55.510998, -381.841, 0.0),
                car_loc: Point3::new(522.79, -383.08, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009561182, 1.9124924, -0.00004696782),
                car_vel: Vector3::new(-472.451, 1328.481, 8.301001),
                enemy_loc: Point3::new(279.72998, 4084.7, 17.01),
                enemy_rot: Rotation3::from_unreal_angles(-0.009587543, -1.7256882, 0.00020834092),
                enemy_vel: Vector3::new(-271.901, -1410.4209, 8.311),
                ..Default::default()
            })
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(4000);

        assert!(test.has_scored());
    }

    #[test]
    fn dont_abandon_shot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1672.57, 2171.4, 342.32),
                ball_rot: Rotation3::from_unreal_angles(-0.29669636, 1.6920353, -2.7599716),
                ball_vel: Vector3::new(-465.701, 715.52094, -324.421),
                ball_ang_vel: Vector3::new(2.9818099, -2.45061, 4.59381),
                car_loc: Point3::new(3026.48, 580.35, 64.7),
                car_rot: Rotation3::from_unreal_angles(0.058283806, -0.6299437, -0.018410105),
                car_vel: Vector3::new(626.96094, -536.03094, -458.631),
                car_ang_vel: Vector3::new(-0.43221, 0.49051, -0.59671),
                ..Default::default()
            })
            .starting_boost(21.0)
            .soccar()
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn dont_miss_a_slightly_bouncing_ball() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2244.3599, 2427.03, 220.15),
                ball_vel: Vector3::new(-174.58101, -757.79095, 104.071),
                car_loc: Point3::new(3708.49, 437.12, 16.98),
                car_rot: Rotation3::from_unreal_angles(-0.00921865, 2.6721888, -0.00048598694),
                car_vel: Vector3::new(-679.941, 371.951, 8.831),
                ..Default::default()
            })
            .starting_boost(99.0)
            .soccar()
            .run_for_millis(3000);

        assert!(test.has_scored());
    }

    #[test]
    fn dont_give_up_when_ball_prediction_has_a_blip() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2550.02, 2887.8499, 1467.25),
                ball_vel: Vector3::new(863.03094, 1751.7009, 172.091),
                car_loc: Point3::new(-3703.5498, 2287.02, 16.52),
                car_rot: Rotation3::from_unreal_angles(-0.0016736704, 0.44723073, 0.013854827),
                car_vel: Vector3::new(1167.011, 648.27094, 18.271),
                ..Default::default()
            })
            .starting_boost(50.0)
            .soccar()
            .run_for_millis(3500);

        assert!(test.has_scored());
    }
}
