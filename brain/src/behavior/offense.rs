use crate::{
    behavior::{
        offense2::reset_behind_ball::ResetBehindBall, shoot::Shoot, tepid_hit::TepidHit, Action,
        Behavior,
    },
    plan::ball::BallFrame,
    predict::naive_ground_intercept_2,
    routing::{behavior::FollowRoute, plan::GetDollar},
    strategy::{Context, Game, Scenario},
    utils::geometry::RayCoordinateSystem,
};
use common::prelude::*;
use nalgebra::Point2;
use std::f32::consts::PI;

pub struct Offense;

impl Offense {
    pub fn new() -> Self {
        Offense
    }
}

impl Behavior for Offense {
    fn name(&self) -> &str {
        stringify!(Offense)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if can_we_shoot(ctx) {
            ctx.eeg.log("[Offense] Taking the shot!");
            return Action::call(Shoot::new());
        }

        // TODO: if angle is almost good, slightly adjust path such that good_angle
        // becomes true

        // TODO: otherwise drive to a point where me.y < ball.y, then slam the ball
        // sideways

        return_some!(slow_play(ctx));

        ctx.eeg.log("[Offense] no good hit; going for a tepid hit");
        return Action::call(TepidHit::new());
    }
}

fn can_we_shoot(ctx: &mut Context) -> bool {
    let me = ctx.me();

    if playing_goalie(ctx.game, ctx.scenario.ball_prediction().start()) {
        return false;
    }

    let naive_intercept = some_or_else!(ctx.scenario.me_intercept(), {
        return false;
    });

    let shoot_intercept = naive_ground_intercept_2(
        &me.into(),
        ctx.scenario.ball_prediction().iter_step_by(0.125),
        |ball| Shoot::viable_shot(ctx.game, me.Physics.loc(), ball.loc),
    );

    let shoot_intercept = some_or_else!(shoot_intercept, {
        return false;
    });

    // Don't just sit there for days waiting for the ball to roll. The more
    // possession we have, the longer we're willing to wait.
    if shoot_intercept.time
        >= naive_intercept.time + ctx.scenario.possession() - Scenario::POSSESSION_CONTESTABLE
    {
        ctx.eeg
            .log("[can_we_shoot] we can shoot, but not soon enough");
        return false;
    }

    true
}

fn playing_goalie(game: &Game, ball: &BallFrame) -> bool {
    let safe = game.own_goal().center_2d;
    let danger = game.enemy_goal().center_2d;
    let defensiveness = (ball.loc.to_2d() - danger).norm() / (ball.loc.to_2d() - safe).norm();
    defensiveness >= 5.0
}

fn slow_play(ctx: &mut Context) -> Option<Action> {
    // Only slow play if we have enough time.
    if ctx.scenario.possession() < 2.0 {
        return None;
    }

    let intercept = some_or_else!(ctx.scenario.me_intercept(), {
        return None;
    });
    let ball_loc = intercept.ball_loc.to_2d();

    if let Some(adjust) = readjust_for_shot(ctx, intercept.time) {
        return Some(adjust);
    }

    // Check if we're already behind the ball; if so, no special action is needed.
    let ball_to_goal = RayCoordinateSystem::segment(ball_loc, ctx.game.enemy_goal().center_2d);
    if ball_to_goal.project(ctx.me().Physics.loc_2d()) < 0.0 {
        return None;
    }

    if ctx.me().Boost < 50 {
        ctx.eeg
            .log("[Offense] Getting boost conveniently behind the ball");
        let behind_ball = Point2::new(
            ball_loc.x,
            ball_loc.y + ctx.game.own_goal().center_2d.y.signum() * 2500.0,
        );
        let dollar = GetDollar::new(behind_ball).target_face(ball_loc);
        return Some(Action::call(FollowRoute::new(dollar)));
    }

    ctx.eeg
        .log("[Offense] Swing around behind the ball for a better hit");
    Some(Action::call(
        ResetBehindBall::behind_loc(ball_loc).distance(2000.0),
    ))
}

fn readjust_for_shot(ctx: &mut Context, intercept_time: f32) -> Option<Action> {
    let ball_loc = ctx
        .scenario
        .ball_prediction()
        .at_time(intercept_time + 2.5)
        .unwrap_or_else(|| ctx.scenario.ball_prediction().last())
        .loc
        .to_2d();

    // We failed to shoot above, but if we adjust, maybe we can shoot
    if ball_loc.x.abs() >= 2000.0 || ball_loc.y.abs() >= 3000.0 {
        return None;
    }

    if ctx.game.enemy_goal().shot_angle_2d(ball_loc) >= PI / 4.0 {
        return None;
    }

    ctx.eeg.log("[Offense] re-adjust for a possible shot");
    return Some(Action::call(
        ResetBehindBall::behind_loc(ball_loc).distance(2000.0),
    ));
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::offense::Offense,
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::Runner2,
    };
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};
    use std::f32::consts::PI;

    #[test]
    #[ignore] // TODO
    fn wait_for_curl_around_lip_near_post() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(2972.6848, 1018.38824, 101.33544),
            ball_vel: Vector3::new(-1029.0707, 2168.4673, -61.355755),
            car_loc: Vector3::new(3147.7668, 686.3356, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.689584, 0.0),
            car_vel: Vector3::new(-685.4966, 1531.4093, 83.56691),
            ..Default::default()
        });
        test.set_behavior(Offense::new());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn in_corner_barely_cant_reach() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(2230.9802, 2748.329, 93.14),
            ball_vel: Vector3::new(640.13696, 820.1949, 0.0),
            car_loc: Vector3::new(2847.7441, 1709.6339, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.4079068, 0.0000958738),
            car_vel: Vector3::new(286.62524, 1500.3096, 8.17),
            boost: 0,
            ..Default::default()
        });
        test.set_behavior(Offense::new());
        test.sleep_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn wait_for_ball_to_fall() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-3987.7068, -2086.639, 329.19128),
            ball_vel: Vector3::new(277.659, -238.58536, 992.14404),
            car_loc: Vector3::new(-2913.0967, -3791.6895, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.9806569, -0.0000958738),
            car_vel: Vector3::new(-352.9971, 833.215, 8.34),
            ..Default::default()
        });
        test.set_behavior(Runner2::soccar());
        test.sleep_millis(3000);
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().y >= 1000.0);
    }

    #[test]
    fn shoot_across_the_goal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Vector3::new(-1438.1412, 4697.7017, 92.71),
                ball_vel: Vector3::new(452.51102, -191.12935, 0.0),
                car_loc: Vector3::new(-2771.101, 4448.831, 17.0),
                car_rot: Rotation3::from_unreal_angles(-0.009491506, -0.038637143, -0.0000958738),
                car_vel: Vector3::new(974.10455, -28.867546, 8.429999),
                boost: 0,
                ..Default::default()
            })
            .behavior(Runner2::soccar())
            .run();
        test.sleep_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn awkward_breakaway_shot() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(1463.9786, -1842.5327, 93.15),
            ball_vel: Vector3::new(-1061.1782, 224.63322, 0.0),
            car_loc: Vector3::new(2019.6799, -3689.1953, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 2.376328, 0.0),
            car_vel: Vector3::new(-962.35034, 924.71826, 8.309999),
            boost: 0,
            ..Default::default()
        });
        test.set_behavior(Runner2::soccar());
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
        test.set_behavior(Runner2::soccar());
        test.sleep_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "The great bankruptcy of 2018")]
    fn tepid_hit_from_own_goal() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(2869.6829, -4145.095, 97.65185),
            ball_vel: Vector3::new(-327.0853, 243.21877, -42.864605),
            car_loc: Vector3::new(286.2713, -5031.399, 16.99),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 0.59949887, 0.0),
            car_vel: Vector3::new(1251.7024, 854.6698, 8.411),
            ..Default::default()
        });
        test.set_behavior(Runner2::soccar());
        test.sleep_millis(100);
        test.examine_eeg(|eeg| {
            assert!(eeg
                .log
                .iter()
                .any(|x| x == "[Offense] no good hit; going for a tepid hit"));
            assert!(eeg.log.iter().any(|x| x == "hitting toward enemy goal"));
        });
    }

    #[test]
    fn slow_play_get_boost() {
        // The setup: we have possession and low boost, and we're on the wrong side of
        // the ball to hit it towards the enemy goal.
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(800.0, 800.0, 90.0).coords,
                ball_vel: Vector3::new(2000.0, -1500.0, 0.0),
                car_loc: Point3::new(-500.0, 200.0, 17.01).coords,
                car_rot: Rotation3::from_unreal_angles(0.0, -PI / 3.0, 0.0),
                car_vel: Vector3::new(500.0, -1000.0, 0.0),
                enemy_loc: Point3::new(-4000.0, 0.0, 17.01).coords,
                boost: 31,
                ..Default::default()
            })
            .behavior(Runner2::soccar())
            .run_for_millis(3000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost > 75);
    }

    #[test]
    fn swing_around_and_shoot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Vector3::new(-480.59, -2596.8699, 200.25),
                ball_vel: Vector3::new(439.35098, 703.78094, -100.480995),
                car_loc: Vector3::new(-1425.59, -2333.4, 110.53),
                car_rot: Rotation3::from_unreal_angles(0.4016184, 2.0482836, 0.02543525),
                car_vel: Vector3::new(-233.431, 1108.031, -368.221),
                ..Default::default()
            })
            .starting_boost(70.0)
            .behavior(Runner2::soccar())
            .run_for_millis(7000);

        assert!(test.has_scored());
    }
}
