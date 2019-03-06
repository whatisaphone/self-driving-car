use crate::{
    behavior::{
        defense::defensive_hit,
        higher_order::Chain,
        movement::{GetToFlatGround, QuickJumpAndDodge, Yielder},
        offense::TepidHit,
        strike::GroundedHit,
    },
    eeg::{color, Drawable, Event},
    helpers::{
        ball::BallFrame, drive::rough_time_drive_to_loc, hit_angle::feasible_angle_near,
        intercept::naive_ground_intercept_2,
    },
    routing::{behavior::FollowRoute, models::CarState, plan::GroundIntercept},
    sim::{SimGroundDrive, SimJump},
    strategy::{Action, Behavior, Context, Game, Priority},
    utils::{geometry::Line2, WallRayCalculator},
};
use common::{prelude::*, Distance, Speed};
use nalgebra::{Point2, Point3};
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;
use vec_box::vec_box;

pub struct RetreatingSave {
    chatted: bool,
}

impl RetreatingSave {
    const MAX_BALL_Z: f32 = 150.0;
    const JUMP_TIME: f32 = 0.1;

    pub fn new() -> Self {
        Self { chatted: false }
    }

    pub fn applicable(ctx: &mut Context<'_>) -> Result<(), &'static str> {
        if !Self::goalside(ctx) {
            return Err("not goalside");
        }

        let goal = ctx.game.own_goal();
        let me_forward = ctx.me().Physics.forward_axis_2d();
        if me_forward.angle_to(&ctx.game.own_goal().normal_2d).abs() < PI / 2.0
            && goal.is_y_within_range(ctx.me().Physics.loc().y, ..500.0)
        {
            return Err("not retreating");
        }

        let impending_concede = ctx
            .scenario
            .impending_concede()
            .map(|b| b.t < 5.0)
            .unwrap_or_default();
        if impending_concede {
            ctx.eeg.draw(Drawable::print("concede", color::GREEN));
            return Ok(());
        }

        let back_wall = ctx
            .scenario
            .ball_prediction()
            .iter_step_by(0.125)
            .any(|ball| {
                goal.is_y_within_range(ball.loc.y, ..500.0)
                    && ball.loc.x.abs() < 1500.0
                    && ball.vel.to_2d().to_axis().angle_to(&-goal.normal_2d).abs() < PI / 3.0
            });
        if back_wall {
            ctx.eeg.draw(Drawable::print("back wall", color::GREEN));
            return Ok(());
        }

        Err("no impending doom")
    }

    /// If the ball will end up rolling in front of our goal, treat it as being
    /// just as dangerous as inside the goal.
    pub fn impending_dangerous_ball<'ctx>(ctx: &mut Context<'ctx>) -> Option<&'ctx BallFrame> {
        ctx.scenario.ball_prediction().iter().find(|ball| {
            let goal = ctx.game.own_goal();
            goal.is_y_within_range(ball.loc.y, ..250.0) && ball.loc.x.abs() < goal.max_x
        })
    }

    /// Returns `true` if we're between the ball and our goal.
    fn goalside(ctx: &mut Context<'_>) -> bool {
        let goal_loc = ctx.game.own_goal().center_2d;
        let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
        let me_loc = ctx.me().Physics.loc_2d();

        if ctx.game.own_goal().is_y_within_range(me_loc.y, ..0.0) {
            return true;
        }

        let axis = (ball_loc - goal_loc).to_axis();
        let ball_dist = (ball_loc - goal_loc).dot(&axis);
        let me_dist = (me_loc - goal_loc).dot(&axis);
        me_dist < ball_dist + 500.0
    }
}

impl Behavior for RetreatingSave {
    fn name(&self) -> &str {
        name_of_type!(RetreatingSave)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if let Err(reason) = Self::applicable(ctx) {
            ctx.eeg.log(self.name(), reason);
            return Action::Abort;
        }

        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            ctx.eeg.log(self.name(), "not on flat ground");
            return Action::Abort;
        }

        let plan = some_or_else!(self.intercept(ctx), {
            ctx.eeg.log(self.name(), "no intercept");
            return Action::Abort;
        });

        if self.striking_would_be_better(ctx, &plan) {
            return Action::tail_call(TepidHit::new());
        }

        if rough_time_drive_to_loc(ctx.me(), plan.target_loc) + 3.0 < plan.target_time {
            ctx.eeg
                .log(self.name(), "yeah, I'm not gonna sit around all day");
            return Action::tail_call(Chain::new(Priority::Idle, vec_box![
                FollowRoute::new(GroundIntercept::new()).same_ball_trajectory(true),
                GroundedHit::hit_towards(defensive_hit),
            ]));
        }

        if !self.chatted {
            ctx.quick_chat(0.1, &[
                rlbot::flat::QuickChatSelection::Information_TakeTheShot,
                rlbot::flat::QuickChatSelection::Information_Defending,
                rlbot::flat::QuickChatSelection::Information_GoForIt,
                rlbot::flat::QuickChatSelection::Information_InPosition,
                rlbot::flat::QuickChatSelection::Compliments_GreatPass,
            ]);
            self.chatted = true;
        }

        ctx.eeg.track(Event::RetreatingSave);
        ctx.eeg.draw(Drawable::ghost_ball(plan.intercept_ball_loc));
        ctx.eeg.draw(Drawable::ghost_car_ground(
            plan.target_loc,
            ctx.me().Physics.rot(),
        ));

        if self.time_to_jump(ctx) {
            ctx.eeg.log(self.name(), "the time is now");
            return self.dodge(ctx);
        }

        self.drive(ctx, &plan)
    }
}

impl RetreatingSave {
    fn intercept(&self, ctx: &mut Context<'_>) -> Option<Plan> {
        let own_goal = ctx.game.own_goal();

        let danger = {
            let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
            let ball_vel = ctx.packet.GameBall.Physics.vel_2d();
            WallRayCalculator::calculate(ball_loc, ball_loc + ball_vel)
        };

        let car_loc = ctx.me().Physics.loc_2d();
        let car_vel = ctx.me().Physics.vel_2d();
        let car_forward_axis = ctx.me().Physics.forward_axis_2d();

        let intercept =
            naive_ground_intercept_2(&ctx.me().into(), ctx.scenario.ball_prediction(), |ball| {
                let me_to_ball = car_loc - ball.loc.to_2d();
                let drive_angle = car_forward_axis.angle_to(&(ball.loc.to_2d() - car_loc));
                let acceptable_drive_angle =
                    linear_interpolate(&[500.0, 2000.0], &[PI / 6.0, PI / 4.0], me_to_ball.norm());

                drive_angle.abs() < acceptable_drive_angle
                    && ball.loc.z < Self::MAX_BALL_Z
                    && !own_goal.ball_is_scored_conservative(ball.loc)
            })?;

        let intercept_ball_loc = intercept.ball_loc.to_2d();

        // Clamp to a convenient blocking angle given our current location.
        let target_loc = feasible_angle_near(intercept_ball_loc, car_loc, danger, PI / 6.0);
        // If the convenient angle is not blocking enough of the danger point, force a
        // less convenient angle.
        let block_loc = if (intercept_ball_loc - own_goal.center_2d).norm() >= 1000.0 {
            own_goal.center_2d
        } else {
            danger
        };
        let clamp_angle = linear_interpolate(&[500.0, 2000.0], &[0.0, PI / 3.0], car_vel.norm());
        let target_loc =
            feasible_angle_near(intercept_ball_loc, block_loc, target_loc, clamp_angle);
        // Target a fixed distance away from the ball.
        let target_loc = intercept_ball_loc + (target_loc - intercept_ball_loc).normalize() * 200.0;

        Some(Plan {
            intercept_ball_loc: intercept.ball_loc,
            target_loc,
            target_steer_loc: target_loc,
            target_time: intercept.time,
        })
    }

    fn striking_would_be_better(&self, ctx: &mut Context<'_>, plan: &Plan) -> bool {
        let goal = ctx.game.own_goal();
        let car_forward_axis = ctx.me().Physics.forward_axis_2d();
        let car_to_ball = ctx.packet.GameBall.Physics.loc_2d() - ctx.me().Physics.loc_2d();
        let car_to_intercept = plan.intercept_ball_loc.to_2d() - ctx.me().Physics.loc_2d();

        car_to_intercept.angle_to(&goal.normal_2d).abs() < PI / 2.0
            && car_forward_axis.angle_to(&car_to_intercept).abs() < PI / 3.0
            && car_forward_axis.angle_to(&car_to_ball).abs() < PI / 3.0
    }

    fn drive(&self, ctx: &mut Context<'_>, plan: &Plan) -> Action {
        if self.should_stop(ctx, plan) {
            ctx.eeg.track(Event::RetreatingSaveStopAndWait);
            let throttle = if ctx.me().Physics.vel_2d().norm() >= 100.0 {
                -1.0
            } else {
                0.0
            };
            return Action::Yield(common::halfway_house::PlayerInput {
                Throttle: throttle,
                ..Default::default()
            });
        }

        let (throttle, boost) = self.calc_drive(ctx, plan);
        let start_loc = ctx.me().Physics.loc_2d();
        let start_forward_axis = ctx.me().Physics.forward_axis_2d();
        let theta = start_forward_axis.angle_to(&(plan.target_steer_loc - start_loc));
        Action::Yield(common::halfway_house::PlayerInput {
            Throttle: throttle,
            Steer: (theta * 2.0).max(-1.0).min(1.0),
            Boost: boost && ctx.me().Boost > 0,
            ..Default::default()
        })
    }

    /// If we're already sitting still and the ball is rolling right towards us,
    /// avoid creeping forward slowly and losing territory.
    fn should_stop(&self, ctx: &mut Context<'_>, plan: &Plan) -> bool {
        let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
        let ball_vel = ctx.packet.GameBall.Physics.vel_2d();
        let car_loc = ctx.me().Physics.loc_2d();
        let car_vel = ctx.me().Physics.vel_2d();

        let cross_speed = car_vel.dot(&ball_vel.ortho().to_axis()).abs();
        let pass_proximity = (car_loc - ball_loc).dot(&ball_vel.ortho().to_axis()).abs();
        let cur_angle = ball_vel.angle_to(&(car_loc - ball_loc)).abs();
        let target_angle = ball_vel
            .angle_to(&(plan.target_loc - plan.intercept_ball_loc.to_2d()))
            .abs();

        ctx.eeg.print_value("cross_speed", Speed(cross_speed));
        ctx.eeg.print_distance("pass_proximity", pass_proximity);
        ctx.eeg.print_angle("cur_angle", cur_angle);
        ctx.eeg.print_angle("target_angle", target_angle);

        ctx.packet.GameBall.Physics.loc().z < 120.0
            && ctx.packet.GameBall.Physics.vel().z < 100.0
            && cross_speed < 300.0
            && pass_proximity < 80.0
            && (cur_angle - target_angle).abs() < PI / 6.0
    }

    fn calc_drive(&self, ctx: &mut Context<'_>, plan: &Plan) -> (f32, bool) {
        let drive_time = plan.target_time - Self::JUMP_TIME;
        if drive_time < 0.0 {
            // Sit tight and wait for the ball to come to us.
            return (-1.0, false);
        }

        let drive = SimGroundDrive::new(plan.target_loc);
        let jump = SimJump;

        let axis = (plan.target_steer_loc - ctx.me().Physics.loc_2d()).to_axis();

        let calc_offset = |throttle, boost| {
            let state = ctx.me().into();
            let state = drive.simulate(&state, drive_time, throttle, boost);
            let state = jump.simulate(&state, Self::JUMP_TIME, &state.rot);

            // Return the distance ahead of the target location.
            (state.loc_2d() - plan.target_loc).dot(&axis)
        };

        let coast_offset = calc_offset(0.0, false);
        let throttle_offset = calc_offset(1.0, false);
        let boost_offset = calc_offset(1.0, true);

        let target_offset = 0.0;

        let (throttle, boost) = if coast_offset >= target_offset + 50.0 {
            (-1.0, false)
        } else if throttle_offset >= target_offset {
            (0.0, false)
        } else if boost_offset >= target_offset {
            (1.0, false)
        } else {
            (1.0, true)
        };

        ctx.eeg.print_value("target", plan.target_loc);
        ctx.eeg.print_time("drive_time", drive_time);
        ctx.eeg.print_time("target_time", plan.target_time);
        ctx.eeg.print_value("coast_offset", Distance(coast_offset));
        ctx.eeg
            .print_value("throttle_offset", Distance(throttle_offset));
        ctx.eeg.print_value("boost_offset", Distance(boost_offset));

        (throttle, boost)
    }

    fn time_to_jump(&self, ctx: &mut Context<'_>) -> bool {
        let (ball, car) = self.simulate_jump(ctx);

        if (ball.loc.to_2d() - car.loc_2d()).norm() >= 300.0 {
            return false;
        }
        // Also wait for the ball to be close enough to the ground, otherwise we'll flip
        // underneath it. We're guaranteed the ball will be there soon because the
        // intercept checker also checks this.
        let max_z = linear_interpolate(
            &[-500.0, 0.0],
            &[Self::MAX_BALL_Z + 50.0, Self::MAX_BALL_Z],
            ball.vel.z,
        );
        if ball.loc.z >= max_z {
            return false;
        }
        return true;
    }

    fn simulate_jump<'ctx>(&self, ctx: &mut Context<'ctx>) -> (&'ctx BallFrame, CarState) {
        // Simulate the ball motion.
        let ball = ctx
            .scenario
            .ball_prediction()
            .at_time(Self::JUMP_TIME)
            .unwrap();

        // Simulate the car motion.
        let apex = SimJump.simulate(&ctx.me().into(), Self::JUMP_TIME, &ctx.me().Physics.quat());

        (ball, apex)
    }

    fn dodge(&self, ctx: &mut Context<'_>) -> Action {
        let (ball, _apex) = self.simulate_jump(ctx);

        if Self::safer_not_to_dodge(
            ctx.game,
            ctx.scenario.ball_prediction().start(),
            &ctx.me().into(),
            ball,
        ) {
            return Action::tail_call(Yielder::new(
                Self::JUMP_TIME,
                common::halfway_house::PlayerInput {
                    Throttle: 0.1,
                    ..Default::default()
                },
            ));
        }

        Action::tail_call(
            QuickJumpAndDodge::new()
                .jump_time(Self::JUMP_TIME)
                .towards_ball(),
        )
    }

    /// In a fast-retreating situation, we might want to avoid dodging so we:
    ///
    ///   - Keep control of our car.
    ///   - Don't propel the ball any faster towards our net than we need to.
    pub fn safer_not_to_dodge(
        game: &Game<'_>,
        ball: &BallFrame,
        car: &CarState,
        contact_ball_frame: &BallFrame,
    ) -> bool {
        // Always dodge if the ball is close to net; in this case there's no control to
        // be preserved, and we want to change the ball's direction as much as possible.
        let ball_vel_dir = Line2::from_origin_dir(ball.loc.to_2d(), ball.vel.to_2d().to_axis());
        let concede_loc = game.own_goal().goalline().intersect(ball_vel_dir).unwrap();
        let concede_time = (concede_loc - ball.loc.to_2d()).norm() / ball.vel.to_2d().norm();
        if concede_time < 2.0 {
            return false;
        }

        let blocking_angle = (game.own_goal().center_2d - ball.loc.to_2d())
            .angle_to(&(car.loc.to_2d() - ball.loc.to_2d()))
            .abs();

        contact_ball_frame.loc.z < 120.0
            && contact_ball_frame.vel.z < 300.0
            && contact_ball_frame.vel.to_2d().norm() >= 1000.0
            && car.vel_2d().dot(&-game.own_goal().normal_2d) >= 1000.0
            && blocking_angle < PI * (4.0 / 8.0)
    }
}

struct Plan {
    intercept_ball_loc: Point3<f32>,
    target_loc: Point2<f32>,
    target_steer_loc: Point2<f32>,
    target_time: f32,
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        eeg::Event,
        integration_tests::{TestRunner, TestScenario},
        strategy::SOCCAR_GOAL_BLUE,
    };
    use brain_test_data::recordings;
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    fn catching_up_to_the_play() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::CATCHING_UP_TO_THE_PLAY, 217.5)
            .starting_boost(15.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x < -2500.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn landing_awkwardly_close_to_the_ball() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::LANDING_AWKWARDLY_CLOSE_TO_THE_BALL, 218.5)
            .starting_boost(15.0)
            .soccar()
            .run_for_millis(3000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!((SOCCAR_GOAL_BLUE.center_2d - ball_loc.to_2d()).norm() >= 2500.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn driving_alongside_rolling_ball() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2184.45, -2221.26, 93.15),
                ball_vel: Vector3::new(966.7009, -1807.241, 0.0),
                car_loc: Point3::new(-1642.62, -2085.82, 39.87),
                car_rot: Rotation3::from_unreal_angles(0.42586005, -0.9628886, -0.009007198),
                car_vel: Vector3::new(1292.1609, -1889.2809, -225.61101),
                ..Default::default()
            })
            .starting_boost(10.0)
            .soccar()
            .run_for_millis(2000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x < -1000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn anticipate_shot() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::ANTICIPATE_SHOT, 321.0)
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(6000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!((ball_loc.to_2d() - SOCCAR_GOAL_BLUE.center_2d).norm() >= 1000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn facing_slightly_away() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(672.2, -2429.41, 93.15),
                ball_rot: Rotation3::from_unreal_angles(-0.23775424, 1.7249714, -2.5660312),
                ball_vel: Vector3::new(-54.611, -2040.121, 0.0),
                car_loc: Point3::new(83.63, -2711.49, 39.01),
                car_rot: Rotation3::from_unreal_angles(0.39634022, -1.8297973, 0.009631016),
                car_vel: Vector3::new(-540.47095, -2108.1008, -234.171),
                ..Default::default()
            })
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(2000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x >= 1500.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn switch_sides_to_goalside() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1243.13, -730.49, 93.15),
                ball_rot: Rotation3::from_unreal_angles(-0.1369289, -2.3818758, 2.5309741),
                ball_vel: Vector3::new(80.321, -1693.861, 0.0),
                car_loc: Point3::new(1277.12, -2066.16, 17.0),
                car_rot: Rotation3::from_unreal_angles(-0.009600522, -1.3817543, -0.00002118205),
                car_vel: Vector3::new(435.031, -2252.741, 8.411),
                enemy_loc: Point3::new(1301.86, -526.81, 17.01),
                enemy_rot: Rotation3::from_unreal_angles(-0.009597596, -1.5676318, -0.00014488742),
                enemy_vel: Vector3::new(-36.440998, -1721.351, 8.331),
                ..Default::default()
            })
            .starting_boost(18.0)
            .soccar()
            .run_for_millis(3000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!((ball_loc.to_2d() - SOCCAR_GOAL_BLUE.center_2d).norm() >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn slow_dribble_behind_us() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::SLOW_DRIBBLE_BEHIND_US, 154.0)
            .starting_boost(70.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!((ball_loc.to_2d() - SOCCAR_GOAL_BLUE.center_2d).norm() >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            // I wish this was yes, but right now it's no:
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn turn_for_bouncing_ball() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::TURN_FOR_BOUNCING_BALL, 30.0)
            .starting_boost(70.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!((ball_loc.to_2d() - SOCCAR_GOAL_BLUE.center_2d).norm() >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    #[ignore(note = "oops, this got broken at some point")]
    fn waiting_awkward_close_to_goal() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::WAITING_AWKWARD_CLOSE_TO_GOAL, 372.0)
            .starting_boost(70.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!((ball_loc.to_2d() - SOCCAR_GOAL_BLUE.center_2d).norm() >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
        });
    }

    #[test]
    fn save_falling_ball() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2795.05, 401.83, 1006.6),
                ball_rot: Rotation3::from_unreal_angles(-0.7367065, -0.9589971, -1.6798002),
                ball_vel: Vector3::new(-191.47101, -628.41095, -200.411),
                ball_ang_vel: Vector3::new(5.69761, -1.73751, 0.71920997),
                car_loc: Point3::new(2131.5, 4.71, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009682266, -1.1720884, -0.0000025035126),
                car_vel: Vector3::new(292.951, -699.36096, 8.311),
                car_ang_vel: Vector3::new(-0.00030999997, 0.00010999999, 0.26981002),
                enemy_loc: Point3::new(2864.0698, 537.70996, 949.43),
                enemy_rot: Rotation3::from_unreal_angles(-0.07544123, -2.1849675, -2.0632482),
                enemy_vel: Vector3::new(-482.441, -1338.9609, 337.441),
                enemy_ang_vel: Vector3::new(0.28061, -1.52221, 2.8947098),
                ..Default::default()
            })
            .starting_boost(60.0)
            .soccar()
            .run_for_millis(2000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn retreating_save_patience() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::RETREATING_SAVE_PATIENCE, 262.0)
            .starting_boost(60.0)
            .soccar()
            .run_for_millis(5000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_vel = packet.GameBall.Physics.vel();
        println!("ball_vel = {:?}", ball_vel);
        assert!(ball_vel.y >= 0.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn make_save_after_turning() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2794.06, -4137.35, 162.48999),
                ball_vel: Vector3::new(-759.18097, -279.371, 466.541),
                ball_ang_vel: Vector3::new(1.8702099, -5.08141, 2.58471),
                car_loc: Point3::new(2089.41, -4659.6997, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009588487, 1.8938375, -0.00021732296),
                car_vel: Vector3::new(-164.481, 353.521, 8.331),
                car_ang_vel: Vector3::new(0.00151, -0.00040999998, 1.78911),
                ..Default::default()
            })
            .starting_boost(60.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_vel = packet.GameBall.Physics.vel();
        println!("ball_vel = {:?}", ball_vel);
        assert!(ball_vel.y >= 500.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn make_save_without_turning() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2492.3499, -4248.4, 295.04),
                ball_vel: Vector3::new(-749.71094, -276.011, 202.241),
                ball_ang_vel: Vector3::new(1.8702099, -5.08141, 2.58471),
                car_loc: Point3::new(1921.58, -4506.73, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00950916, 2.79175, -0.000023257693),
                car_vel: Vector3::new(-704.91095, 251.911, 8.331),
                ..Default::default()
            })
            .starting_boost(60.0)
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            // I wish this was yes, but right now it's no:
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }

    #[test]
    fn make_bouncing_save() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2597.49, 425.9, 1656.75),
                ball_vel: Vector3::new(-808.73096, -1761.6709, -650.251),
                ball_ang_vel: Vector3::new(-4.38501, 2.74741, -3.03681),
                car_loc: Point3::new(2146.41, -2281.45, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009603632, -1.7794317, 0.0004454904),
                car_vel: Vector3::new(-261.681, -1001.001, 8.331),
                car_ang_vel: Vector3::new(0.00021, -0.00061, -1.16751),
                ..Default::default()
            })
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(4500);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_vel = packet.GameBall.Physics.vel();
        println!("ball_vel = {:?}", ball_vel);
        assert!(ball_vel.y >= 500.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::RetreatingSave));
            assert!(!events.contains(&Event::RetreatingSaveStopAndWait));
        });
    }
}
