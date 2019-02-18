use crate::{
    behavior::offense::{ResetBehindBall, Shoot, TepidHit},
    eeg::Event,
    helpers::{ball::BallFrame, intercept::naive_ground_intercept_2, telepathy},
    routing::{behavior::FollowRoute, plan::GetDollar},
    strategy::{Action, Behavior, Context, Game, Scenario},
    utils::geometry::RayCoordinateSystem,
};
use common::{prelude::*, Angle, Distance};
use nalgebra::{Point2, Vector2};
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;

pub struct Offense;

impl Offense {
    pub fn new() -> Self {
        Self
    }
}

impl Behavior for Offense {
    fn name(&self) -> &str {
        name_of_type!(Offense)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg.track(Event::Offense);

        if can_we_shoot(ctx) {
            ctx.eeg.log(self.name(), "taking the shot!");
            ctx.quick_chat(0.1, &[
                rlbot::flat::QuickChatSelection::Information_Incoming,
            ]);
            return Action::tail_call(Shoot::new());
        }

        // TODO: if angle is almost good, slightly adjust path such that good_angle
        // becomes true

        // TODO: otherwise drive to a point where me.y < ball.y, then slam the ball
        // sideways

        if let Some(action) = slow_play(ctx) {
            ctx.quick_chat(0.01, &[rlbot::flat::QuickChatSelection::Information_IGotIt]);
            return action;
        }

        if let Some(b) = get_boost(ctx) {
            ctx.quick_chat(0.1, &[
                rlbot::flat::QuickChatSelection::Information_NeedBoost,
            ]);
            return Action::TailCall(b);
        }

        if let Some(action) = poor_angle_swing_around(ctx) {
            ctx.quick_chat(0.01, &[rlbot::flat::QuickChatSelection::Information_IGotIt]);
            return action;
        }

        ctx.eeg
            .log(self.name(), "no good hit; going for a tepid hit");
        Action::tail_call(TepidHit::new())
    }
}

fn can_we_shoot(ctx: &mut Context<'_>) -> bool {
    let me = ctx.me();

    if playing_goalie(ctx.game, ctx.scenario.ball_prediction().start()) {
        ctx.eeg
            .log(name_of_type!(Offense), "can_we_shoot: playing goalie");
        return false;
    }

    let naive_intercept = some_or_else!(ctx.scenario.me_intercept(), {
        ctx.eeg
            .log(name_of_type!(Offense), "can_we_shoot: no intercept");
        return false;
    });

    let shoot_intercept = naive_ground_intercept_2(
        &me.into(),
        ctx.scenario.ball_prediction().iter_step_by(0.125),
        |ball| Shoot::viable_shot(ctx.game, me.Physics.loc(), ball.loc),
    );

    let shoot_intercept = some_or_else!(shoot_intercept, {
        ctx.eeg.log(
            name_of_type!(Offense),
            "can_we_shoot: no shootable intercept",
        );
        return false;
    });

    let naive_intercept = naive_intercept.time.min(shoot_intercept.time);

    let acceptable_delay = if ctx.scenario.possession() >= 2.0 {
        2.0
    } else {
        0.5
    };

    // Don't just sit there for days waiting for the ball to roll. The more
    // possession we have, the longer we're willing to wait.
    if shoot_intercept.time >= naive_intercept + acceptable_delay {
        ctx.eeg.log(
            name_of_type!(Offense),
            "can_we_shoot: yes but not soon enough",
        );
        return false;
    }

    true
}

fn playing_goalie(game: &Game<'_>, ball: &BallFrame) -> bool {
    let safe = game.own_goal().center_2d;
    let danger = game.enemy_goal().center_2d;
    let defensiveness = (ball.loc.to_2d() - danger).norm() / (ball.loc.to_2d() - safe).norm();
    defensiveness >= 5.0
}

fn slow_play(ctx: &mut Context<'_>) -> Option<Action> {
    // Only slow play if we have enough time.
    if ctx.scenario.possession() < 2.0 {
        ctx.eeg
            .log(name_of_type!(Offense), "slow_play: need possession");
        return None;
    }

    let intercept = ctx.scenario.me_intercept()?;
    let ball_loc = intercept.ball_loc.to_2d();

    if let Some(adjust) = readjust_for_shot(ctx, intercept.time) {
        return Some(adjust);
    }

    // Check if we're already behind the ball; if so, no special action is needed.
    let ball_to_goal = RayCoordinateSystem::segment(ball_loc, ctx.game.enemy_goal().center_2d);
    if ball_to_goal.project(ctx.me().Physics.loc_2d()) < 0.0 {
        ctx.eeg
            .log(name_of_type!(Offense), "slow_play: already behind the ball");
        return None;
    }

    if ctx.me().Boost < 50 {
        ctx.eeg.log(
            name_of_type!(Offense),
            "slow_play: getting boost conveniently behind the ball",
        );
        let behind_ball = Point2::new(
            ball_loc.x,
            ball_loc.y + ctx.game.own_goal().center_2d.y.signum() * 2500.0,
        );
        let dollar = GetDollar::new(behind_ball).target_face(ball_loc);
        return Some(Action::tail_call(FollowRoute::new(dollar)));
    }

    ctx.eeg.log(name_of_type!(Offense), "slow_play: proceeding");
    Some(Action::tail_call(ResetBehindBall::behind_loc(
        ball_loc, 2000.0,
    )))
}

fn readjust_for_shot(ctx: &mut Context<'_>, intercept_time: f32) -> Option<Action> {
    let ball_loc = ctx
        .scenario
        .ball_prediction()
        .at_time_or_last(intercept_time + 2.5)
        .loc
        .to_2d();

    // We failed to shoot above, but if we adjust, maybe we can shoot
    if ball_loc.x.abs() >= 2000.0 || ball_loc.y.abs() >= 3000.0 {
        ctx.eeg.log(
            name_of_type!(Offense),
            "readjust_for_shot: too close to edge of field",
        );
        return None;
    }

    if ctx.game.enemy_goal().shot_angle_2d(ball_loc) >= PI / 4.0 {
        ctx.eeg.log(
            name_of_type!(Offense),
            "readjust_for_shot: not a good enough angle",
        );
        return None;
    }

    ctx.eeg
        .log(name_of_type!(Offense), "readjust_for_shot: proceeding");
    let distance = reset_distance(ctx, ball_loc);
    Some(Action::tail_call(ResetBehindBall::behind_loc(
        ball_loc, distance,
    )))
}

fn get_boost(ctx: &mut Context<'_>) -> Option<Box<dyn Behavior>> {
    if ctx.me().Boost > 50 {
        ctx.eeg
            .log(name_of_type!(Offense), "get_boost: already have enough");
        return None;
    }
    if ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE
        && ctx.scenario.enemy_shoot_score_seconds() >= 7.0
    {
        ctx.eeg.log(
            name_of_type!(Offense),
            format!(
                "enemy_shoot_score_seconds is {:.2}, so let's get boost",
                ctx.scenario.enemy_shoot_score_seconds(),
            ),
        );

        let future_loc = ctx.scenario.ball_prediction().at_time_or_last(3.0).loc;
        let behind_ball = Vector2::new(0.0, ctx.game.own_goal().center_2d.y.signum() * 2500.0);
        let opponent_hit = telepathy::predict_enemy_hit_direction(ctx)
            .map(|dir| dir.into_inner() * 2500.0)
            .unwrap_or_else(Vector2::zeros);
        let hint = future_loc.to_2d() + behind_ball + opponent_hit;
        ctx.eeg
            .log_pretty(name_of_type!(Offense), "opponent_hit", opponent_hit);
        ctx.eeg.log_pretty(name_of_type!(Offense), "hint", hint);
        let get_dollar = GetDollar::new(hint).target_face(future_loc.to_2d());
        return Some(Box::new(FollowRoute::new(get_dollar)));
    }
    None
}

fn poor_angle_swing_around(ctx: &mut Context<'_>) -> Option<Action> {
    let goal_loc = ctx.game.enemy_goal().center_2d;
    let me_loc = ctx.me().Physics.loc_2d();
    let me_intercept = ctx.scenario.me_intercept()?;
    let ball_loc = me_intercept.ball_loc.to_2d();

    // If it's a tap in, don't lose focus of tapping it in.
    if (goal_loc.x - ball_loc.x).abs() < ctx.game.enemy_goal().max_x
        && (goal_loc.y - ball_loc.y).abs() < 500.0
    {
        ctx.eeg.log(
            name_of_type!(Offense),
            "poor_angle_swing_around: rather just tap it in",
        );
        return None;
    }

    let shot_angle = (ball_loc - me_loc)
        .angle_to(&(goal_loc - ball_loc).to_axis())
        .abs();
    if shot_angle < 135.0_f32.to_radians() {
        ctx.eeg.log(
            name_of_type!(Offense),
            "poor_angle_swing_around: angle not poor enough",
        );
        return None;
    }

    if ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE {
        ctx.eeg.log(
            name_of_type!(Offense),
            "poor_angle_swing_around: no possession",
        );
        return None;
    }

    ctx.eeg.log(
        name_of_type!(Offense),
        "poor_angle_swing_around: proceeding",
    );
    let future_ball = ctx
        .scenario
        .ball_prediction()
        .at_time_or_last(me_intercept.time.max(2.5));
    let distance = reset_distance(ctx, future_ball.loc.to_2d());
    Some(Action::tail_call(ResetBehindBall::behind_loc(
        future_ball.loc.to_2d(),
        distance,
    )))
}

fn reset_distance(ctx: &mut Context<'_>, ball_loc: Point2<f32>) -> f32 {
    // Choose how far to back up. If we're retreating, we can turn on a dime
    // (powerslide), so we don't need as much space. If we're moving across the
    // field, we'll be moving faster and we'll need more space to turn.
    let goal = ctx.game.enemy_goal();
    let me_loc = ctx.me().Physics.loc().to_2d();
    let behind_ball_loc = ball_loc + (ball_loc - goal.center_2d).normalize() * 1000.0;
    let shot_angle = (behind_ball_loc - me_loc).angle_to(&goal.normal_2d).abs();
    let distance = linear_interpolate(&[PI / 4.0, PI / 2.0], &[1000.0, 2000.0], shot_angle);

    ctx.eeg.log_pretty(
        name_of_type!(Offense),
        "reset_distance shot_angle",
        Angle(shot_angle),
    );
    ctx.eeg
        .log_pretty(name_of_type!(Offense), "reset_distance", Distance(distance));

    distance
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::offense::Offense,
        eeg::Event,
        integration_tests::{TestRunner, TestScenario},
    };
    use brain_test_data::recordings;
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};
    use std::f32::consts::PI;

    #[test]
    #[ignore(note = "TODO")]
    fn wait_for_curl_around_lip_near_post() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2972.6848, 1018.38824, 101.33544),
                ball_vel: Vector3::new(-1029.0707, 2168.4673, -61.355755),
                car_loc: Point3::new(3147.7668, 686.3356, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.689584, 0.0),
                car_vel: Vector3::new(-685.4966, 1531.4093, 83.56691),
                ..Default::default()
            })
            .behavior(Offense::new())
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "TODO")]
    fn in_corner_barely_cant_reach() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2230.9802, 2748.329, 93.14),
                ball_vel: Vector3::new(640.13696, 820.1949, 0.0),
                car_loc: Point3::new(2847.7441, 1709.6339, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.4079068, 0.0000958738),
                car_vel: Vector3::new(286.62524, 1500.3096, 8.17),
                boost: 0,
                ..Default::default()
            })
            .behavior(Offense::new())
            .run_for_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    fn wait_for_ball_to_fall() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-3987.7068, -2086.639, 329.19128),
                ball_vel: Vector3::new(277.659, -238.58536, 992.14404),
                car_loc: Point3::new(-2913.0967, -3791.6895, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.9806569, -0.0000958738),
                car_vel: Vector3::new(-352.9971, 833.215, 8.34),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3500);
        let packet = test.sniff_packet();
        println!("ball vel = {:?}", packet.GameBall.Physics.vel(),);
        assert!(packet.GameBall.Physics.vel().y >= 100.0);
        assert!(packet.GameBall.Physics.vel().norm() >= 500.0);
    }

    #[test]
    fn shoot_across_the_goal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1438.1412, 4697.7017, 92.71),
                ball_vel: Vector3::new(452.51102, -191.12935, 0.0),
                car_loc: Point3::new(-2771.101, 4448.831, 17.0),
                car_rot: Rotation3::from_unreal_angles(-0.009491506, -0.038637143, -0.0000958738),
                car_vel: Vector3::new(974.10455, -28.867546, 8.429999),
                boost: 0,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    fn awkward_breakaway_shot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1463.9786, -1842.5327, 93.15),
                ball_vel: Vector3::new(-1061.1782, 224.63322, 0.0),
                car_loc: Point3::new(2019.6799, -3689.1953, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 2.376328, 0.0),
                car_vel: Vector3::new(-962.35034, 924.71826, 8.309999),
                boost: 0,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(8000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "TODO")]
    fn juicy_bouncing() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(3811.8657, 1580.2241, 1172.8545),
                ball_vel: Vector3::new(-1703.4757, 753.66534, 210.33441),
                car_loc: Point3::new(-1648.8497, 1804.7543, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009779127, 1.909902, 0.0000958738),
                car_vel: Vector3::new(-702.66034, 1446.7336, 8.615206),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3000);
        assert!(test.has_scored());
    }

    #[test]
    fn tepid_hit_from_own_goal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2869.6829, -4145.095, 97.65185),
                ball_vel: Vector3::new(-327.0853, 243.21877, -42.864605),
                car_loc: Point3::new(286.2713, -5031.399, 16.99),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 0.59949887, 0.0),
                car_vel: Vector3::new(1251.7024, 854.6698, 8.411),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(100);

        test.examine_events(|events| {
            assert!(events.contains(&Event::TepidHitTowardEnemyGoal));
        });
    }

    #[test]
    fn slow_play_get_boost() {
        // The setup: we have possession and low boost, and we're on the wrong side of
        // the ball to hit it towards the enemy goal.
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(800.0, 800.0, 90.0),
                ball_vel: Vector3::new(2000.0, -1500.0, 0.0),
                car_loc: Point3::new(-500.0, 200.0, 17.01),
                car_rot: Rotation3::from_unreal_angles(0.0, -PI / 3.0, 0.0),
                car_vel: Vector3::new(500.0, -1000.0, 0.0),
                enemy_loc: Point3::new(-4000.0, 0.0, 17.01),
                boost: 31,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost > 75);
    }

    #[test]
    fn swing_around_and_shoot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-480.59, -2596.8699, 200.25),
                ball_vel: Vector3::new(439.35098, 703.78094, -100.480995),
                car_loc: Point3::new(-1425.59, -2333.4, 110.53),
                car_rot: Rotation3::from_unreal_angles(0.4016184, 2.0482836, 0.02543525),
                car_vel: Vector3::new(-233.431, 1108.031, -368.221),
                ..Default::default()
            })
            .starting_boost(70.0)
            .soccar()
            .run_for_millis(7000);

        assert!(test.has_scored());
    }

    #[test]
    fn swing_around_while_retreating_but_not_too_far() {
        let test = TestRunner::new()
            .one_v_one(
                &*recordings::SWING_AROUND_WHILE_RETREATING_BUT_NOT_TOO_FAR,
                70.1,
            )
            .starting_boost(0.0)
            .enemy_starting_boost(40.0)
            .soccar()
            .run_for_millis(5000);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball loc = {:?}", ball_loc);
        assert!(ball_loc.y >= -1000.0);
    }

    #[test]
    fn dont_delay_shot_without_possession() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::DONT_DELAY_SHOT_WITHOUT_POSSESSION, 27.0)
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        let ball_vel = packet.GameBall.Physics.vel();
        println!("ball_vel = {:?}", ball_vel);
        assert!(ball_vel.y >= 0.0);
    }
}
