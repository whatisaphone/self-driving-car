use crate::{
    behavior::movement::QuickJumpAndDodge,
    eeg::Drawable,
    plan::hit_angle::feasible_hit_angle_away,
    predict::naive_ground_intercept_2,
    routing::models::CarState,
    sim::{SimGroundDrive, SimJump},
    strategy::{Action, Behavior, Context},
    utils::{Wall, WallRayCalculator},
};
use common::{prelude::*, Distance};
use nalgebra::{Point2, Point3};
use nameof::name_of_type;
use std::f32::consts::PI;

pub struct RetreatingSave;

impl RetreatingSave {
    const MAX_BALL_Z: f32 = 150.0;
    const JUMP_TIME: f32 = 0.1;

    #[cfg(test)]
    pub fn new() -> Self {
        Self
    }
}

impl Behavior for RetreatingSave {
    fn name(&self) -> &str {
        name_of_type!(RetreatingSave)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let plan = some_or_else!(self.intercept(ctx), {
            ctx.eeg.log(self.name(), "no intercept");
            return Action::Abort;
        });

        ctx.eeg.draw(Drawable::ghost_ball(plan.intercept_ball_loc));
        ctx.eeg.draw(Drawable::ghost_car_ground(
            plan.target_loc,
            ctx.me().Physics.rot(),
        ));

        match self.estimate_drive(ctx, plan.target_loc, plan.target_time) {
            Step::Drive(throttle, boost) => self.drive(ctx, plan.target_loc, throttle, boost),
            Step::Dodge => self.dodge(ctx),
        }
    }
}

impl RetreatingSave {
    fn intercept(&self, ctx: &mut Context<'_>) -> Option<Plan> {
        let own_goal = ctx.game.own_goal();

        let danger = {
            let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
            let ball_vel = ctx.packet.GameBall.Physics.vel_2d();
            let ball_trajectory = WallRayCalculator::calculate(ball_loc, ball_loc + ball_vel);
            own_goal.closest_point(ball_trajectory)
        };

        let car_loc = ctx.me().Physics.loc_2d();
        let car_forward_axis = ctx.me().Physics.forward_axis_2d();

        let intercept =
            naive_ground_intercept_2(&ctx.me().into(), ctx.scenario.ball_prediction(), |ball| {
                let drive_angle = car_forward_axis
                    .angle_to(&(ball.loc.to_2d() - car_loc))
                    .abs();
                let aim_loc = feasible_hit_angle_away(ball.loc.to_2d(), car_loc, danger, PI / 6.0);
                let aim_loc = WallRayCalculator::calculate(ball.loc.to_2d(), aim_loc);
                if WallRayCalculator::wall_for_point(ctx.game, aim_loc) == Wall::OwnGoal {
                    return false;
                }
                drive_angle.abs() < PI / 6.0
                    && ball.loc.z < Self::MAX_BALL_Z
                    && !own_goal.ball_is_scored(ball.loc)
            })?;

        let ball_loc = intercept.ball_loc.to_2d();
        let ball_vel = intercept.ball_vel.to_2d();

        let side = car_forward_axis.angle_to(&(ball_loc - car_loc)).signum();
        let target_angle = ball_vel.to_axis().ortho() * side;
        let target_loc = ball_loc + target_angle.normalize() * 200.0;
        Some(Plan {
            intercept_ball_loc: intercept.ball_loc,
            target_loc,
            target_time: intercept.time,
        })
    }

    fn estimate_drive(
        &mut self,
        ctx: &mut Context<'_>,
        target_loc: Point2<f32>,
        target_time: f32,
    ) -> Step {
        let jump_duration = 0.1;
        let drive_time = target_time - jump_duration;

        if drive_time < 0.0 {
            return Step::Dodge;
        }

        let drive = SimGroundDrive::new(target_loc);
        let jump = SimJump;

        let axis = (target_loc - ctx.me().Physics.loc_2d()).to_axis();

        let calc_offset = |throttle, boost| {
            let state = ctx.me().into();
            let state = drive.simulate(&state, drive_time, throttle, boost);
            let state = jump.simulate(&state, Self::JUMP_TIME, &state.rot);

            // Return the distance ahead of the target location.
            (state.loc_2d() - target_loc).dot(&axis)
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

        ctx.eeg.print_value("target", target_loc);
        ctx.eeg.print_time("drive_time", drive_time);
        ctx.eeg.print_time("target_time", target_time);
        ctx.eeg.print_value("coast_offset", Distance(coast_offset));
        ctx.eeg
            .print_value("throttle_offset", Distance(throttle_offset));
        ctx.eeg.print_value("boost_offset", Distance(boost_offset));

        Step::Drive(throttle, boost)
    }

    fn drive(
        &self,
        ctx: &mut Context<'_>,
        target_loc: Point2<f32>,
        throttle: f32,
        boost: bool,
    ) -> Action {
        let start_loc = ctx.me().Physics.loc_2d();
        let start_forward_axis = ctx.me().Physics.forward_axis_2d();
        let theta = start_forward_axis.angle_to(&(target_loc - start_loc));
        Action::Yield(rlbot::ffi::PlayerInput {
            Throttle: throttle,
            Steer: (theta * 2.0).max(-1.0).min(1.0),
            Boost: boost,
            ..Default::default()
        })
    }

    fn simulate_jump(&self, ctx: &mut Context<'_>) -> (Point2<f32>, CarState) {
        // Simulate the ball motion.
        let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
        let ball_vel = ctx.packet.GameBall.Physics.vel_2d();
        let ball_loc = ball_loc + ball_vel * Self::JUMP_TIME;

        // Simulate the car motion.
        let apex = SimJump.simulate(&ctx.me().into(), Self::JUMP_TIME, &ctx.me().Physics.quat());

        (ball_loc, apex)
    }

    fn dodge(&self, ctx: &mut Context<'_>) -> Action {
        let (ball_loc, apex) = self.simulate_jump(ctx);
        let theta = apex.forward_axis_2d().angle_to(&(ball_loc - apex.loc_2d()));
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
    target_time: f32,
}

enum Step {
    Drive(f32, bool),
    Dodge,
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::defense::retreating_save::RetreatingSave,
        integration_tests::helpers::{TestRunner, TestScenario},
    };
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
            .behavior(RetreatingSave::new())
            .run_for_millis(2000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.x < -1000.0);
    }
}
