use crate::{
    behavior::{
        defense::defensive_hit,
        higher_order::Chain,
        movement::{GetToFlatGround, QuickJumpAndDodge, Yielder},
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
    utils::{geometry::Line2, Wall, WallRayCalculator},
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

    fn applicable(ctx: &mut Context<'_>) -> Result<(), &'static str> {
        if !Self::goalside(ctx) {
            return Err("not goalside");
        }

        let impending_concede = ctx
            .scenario
            .impending_concede()
            .or_else(|| Self::impending_dangerous_ball(ctx))
            .map(|b| b.t < 5.0)
            .unwrap_or_default();
        if impending_concede {
            ctx.eeg.draw(Drawable::print("concede", color::GREEN));
            return Ok(());
        }

        let ball_extrap = WallRayCalculator::calc_from_motion(
            ctx.packet.GameBall.Physics.loc_2d(),
            ctx.packet.GameBall.Physics.vel_2d(),
        );
        match WallRayCalculator::wall_for_point(ctx.game, ball_extrap) {
            Wall::OwnGoal | Wall::OwnBackWall if ball_extrap.x.abs() < 1500.0 => {
                ctx.eeg.draw(Drawable::print("back wall", color::GREEN));
                return Ok(());
            }
            _ => {}
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

        if rough_time_drive_to_loc(ctx.me(), plan.target_loc) + 3.0 < plan.target_time {
            ctx.eeg
                .log(self.name(), "yeah, I'm not gonna sit around all day");
            return Action::tail_call(Chain::new(Priority::Idle, vec_box![
                FollowRoute::new(GroundIntercept::new()),
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

    fn drive(&self, ctx: &mut Context<'_>, plan: &Plan) -> Action {
        if self.should_stop(ctx, plan) {
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

    /// If we're already sitting still and the ball is headed right for us,
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

        cross_speed < 300.0 && pass_proximity < 80.0 && (cur_angle - target_angle).abs() < PI / 6.0
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
        if ball.loc.z >= Self::MAX_BALL_Z {
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
    use common::{prelude::*, rl};
    use nalgebra::{Point2, Point3, Rotation3, Vector3};

    #[test]
    #[ignore(note = "TODO")]
    fn falling_in_front_of_far_corner() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(882.9138, -5002.2944, 608.2664),
                ball_vel: Vector3::new(-211.04604, 37.17434, 459.58438),
                car_loc: Point3::new(-2512.3357, -2450.706, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009683254, -0.68204623, -0.0000958738),
                car_vel: Vector3::new(786.13666, -620.0981, 8.309999),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(2500);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        let own_goal = Point2::new(0.0, -rl::FIELD_MAX_Y);
        let goal_to_ball_dist = (packet.GameBall.Physics.loc_2d() - own_goal).norm();
        assert!(goal_to_ball_dist >= 750.0);
        assert!(packet.GameBall.Physics.vel().norm() >= 1000.0);
    }

    #[test]
    fn rolling_quickly() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2792.5564, 2459.176, 94.02834),
                ball_vel: Vector3::new(-467.7808, -2086.822, -88.445175),
                car_loc: Point3::new(3001.808, 3554.98, 16.99),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.7710767, 0.0000958738),
                car_vel: Vector3::new(-379.28546, -1859.9683, 8.41),
                enemy_loc: Point3::new(3301.808, 3554.98, 16.99),
                enemy_rot: Rotation3::from_unreal_angles(-0.00958738, -1.7710767, 0.0000958738),
                enemy_vel: Vector3::new(-379.28546, -1859.9683, 8.41),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(2500);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().x >= -200.0);
    }

    #[test]
    #[ignore(note = "TODO")]
    fn rolling_around_corner_into_box() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(3042.6016, -4141.044, 180.57321),
                ball_vel: Vector3::new(-1414.86847, -1357.0486, -0.0),
                car_loc: Point3::new(720.54016, 635.665, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.4134674, 0.0),
                car_vel: Vector3::new(256.23804, -1591.1218, 8.3),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn low_bouncing_directly_ahead() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-916.57043, -5028.2397, 449.42386),
                ball_vel: Vector3::new(215.22325, 0.07279097, -403.102),
                car_loc: Point3::new(-320.59094, -2705.4436, 17.02),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.6579456, 0.0),
                car_vel: Vector3::new(-85.847946, -990.35706, 8.0),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3000);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.vel());
        assert!(packet.GameBall.Physics.vel().x < -1000.0);
    }

    #[test]
    #[ignore(note = "TODO")]
    fn high_loft_in_front_of_goal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2285.6035, -5024.131, 438.6606),
                ball_vel: Vector3::new(751.0301, 16.736507, 811.52356),
                car_loc: Point3::new(-1805.5178, -2341.8872, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -0.4485935, 0.0),
                car_vel: Vector3::new(1141.101, -487.77042, 8.34),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn loft_in_front_of_goal_from_the_side() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2288.2634, -4688.248, 93.15),
                ball_vel: Vector3::new(1281.6293, -1659.181, 0.0),
                car_loc: Point3::new(-3077.711, -3389.5276, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -0.95528656, -0.0000958738),
                car_vel: Vector3::new(1027.5283, -1455.2512, 8.3),
                enemy_loc: Point3::new(-1500.0, -4000.0, 17.01),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        println!("loc = {:?}", packet.GameBall.Physics.loc());
        assert!(packet.GameBall.Physics.loc().x >= 1000.0);
        println!("vel = {:?}", packet.GameBall.Physics.vel());
        assert!(packet.GameBall.Physics.vel().x >= 750.0);
    }

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
        });
    }
}
