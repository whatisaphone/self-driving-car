use crate::{
    behavior::movement::{GetToFlatGround, QuickJumpAndDodge, Yielder},
    eeg::{color, Drawable, Event},
    helpers::{hit_angle::feasible_angle_near, intercept::naive_ground_intercept_2},
    routing::models::CarState,
    sim::{SimGroundDrive, SimJump},
    strategy::{Action, Behavior, Context},
    utils::{Wall, WallRayCalculator},
};
use common::{prelude::*, Distance};
use nalgebra::{Point2, Point3};
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;

pub struct RetreatingSave {
    chatted: bool,
}

impl RetreatingSave {
    const MAX_BALL_Z: f32 = 150.0;
    const JUMP_TIME: f32 = 0.1;
    const BALL_Z_FOR_DODGE: f32 = 120.0;

    pub fn new() -> Self {
        Self { chatted: false }
    }

    fn applicable(ctx: &mut Context<'_>) -> Result<(), &'static str> {
        if !Self::goalside(ctx) {
            return Err("not goalside");
        }
        if ctx.scenario.impending_concede().map(|b| b.t < 5.0) == Some(true) {
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

        let (ball_loc, car) = self.simulate_jump(ctx);
        if (ball_loc.to_2d() - car.loc_2d()).norm() < 250.0 {
            ctx.eeg.log(self.name(), "we are close enough");
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
                    linear_interpolate(&[500.0, 2000.0], &[PI / 6.0, PI / 3.0], me_to_ball.norm());

                drive_angle.abs() < acceptable_drive_angle
                    && ball.loc.z < Self::MAX_BALL_Z
                    && !own_goal.ball_is_scored(ball.loc)
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

    fn drive(&self, ctx: &mut Context<'_>, plan: &Plan) -> Action {
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

    fn simulate_jump(&self, ctx: &mut Context<'_>) -> (Point3<f32>, CarState) {
        // Simulate the ball motion.
        let ball_loc = ctx.packet.GameBall.Physics.loc();
        let ball_vel = ctx.packet.GameBall.Physics.vel();
        let ball_loc = ball_loc + ball_vel * Self::JUMP_TIME;

        // Simulate the car motion.
        let apex = SimJump.simulate(&ctx.me().into(), Self::JUMP_TIME, &ctx.me().Physics.quat());

        (ball_loc, apex)
    }

    fn dodge(&self, ctx: &mut Context<'_>) -> Action {
        let (ball_loc, apex) = self.simulate_jump(ctx);

        // This is a dangerous situation as it is. Skip the dodge if we can, because we
        // want to:
        // - Keep control of our car.
        // - Not propel the ball any faster towards our net than we need to.
        if ball_loc.z < Self::BALL_Z_FOR_DODGE && ctx.packet.GameBall.Physics.vel().norm() >= 2000.0
        {
            return Action::tail_call(Yielder::new(
                common::halfway_house::PlayerInput {
                    Throttle: 0.1,
                    ..Default::default()
                },
                Self::JUMP_TIME,
            ));
        }

        let theta = apex
            .forward_axis_2d()
            .angle_to(&(ball_loc.to_2d() - apex.loc_2d()));
        Action::tail_call(
            QuickJumpAndDodge::new()
                .jump_time(Self::JUMP_TIME)
                .angle(theta),
        )
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
}
