use behavior::{Behavior, Chain, Defense, Offense, Priority};
use common::prelude::*;
use maneuvers::{FiftyFifty, GetToFlatGround};
use routing::{behavior::FollowRoute, plan::GetDollar};
use std::f32::consts::PI;
use strategy::{scenario::Scenario, strategy::Strategy, Context};
use utils::{geometry::ExtendF32, my_goal_center_2d, Wall};

#[derive(new)]
pub struct Soccar;

impl Strategy for Soccar {
    fn baseline(&mut self, ctx: &mut Context) -> Box<Behavior> {
        if !GetToFlatGround::on_flat_ground(ctx.packet) {
            return Box::new(GetToFlatGround::new());
        }

        match ctx.scenario.push_wall() {
            Wall::OwnGoal | Wall::OwnBackWall => return Box::new(Defense::new()),
            _ => {}
        }

        return_some!(get_boost(ctx));

        Box::new(Offense::new())
    }

    fn interrupt(&mut self, ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>> {
        // Force kickoff behavior
        if current.priority() < Priority::Force
            && ctx.packet.GameBall.Physics.loc().norm() < 1.0
            && ctx.packet.GameBall.Physics.vel().norm() < 1.0
        {
            return Some(Box::new(Chain::new(
                Priority::Force,
                vec![Box::new(FiftyFifty::new())],
            )));
        }

        if current.priority() < Priority::Save && enemy_can_shoot(ctx) {
            if ctx.scenario.possession().abs() < Scenario::POSSESSION_CONTESTABLE {
                ctx.eeg.log(format!(
                    "enemy can shoot, possession = {:.2}, going for 50/50",
                    ctx.scenario.possession()
                ));
                return Some(Box::new(Chain::new(
                    Priority::Save,
                    vec![
                        Box::new(GetToFlatGround::new()),
                        Box::new(FiftyFifty::new()),
                    ],
                )));
            }

            if ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE {
                ctx.eeg.log(format!(
                    "enemy can shoot, possession = {:.2}, going to defense",
                    ctx.scenario.possession()
                ));
                return Some(Box::new(Chain::new(
                    Priority::Save,
                    vec![Box::new(Defense::new())],
                )));
            }
        }

        None
    }
}

fn get_boost(ctx: &mut Context) -> Option<Box<Behavior>> {
    if ctx.me().Boost > 50 {
        return None;
    }
    if ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE {
        if ctx.scenario.enemy_shoot_score_seconds() >= 7.0 {
            return Some(Box::new(FollowRoute::new(GetDollar::new(
                ctx.game.own_goal().center_2d,
            ))));
        }
    }
    return None;
}

fn enemy_can_shoot(ctx: &mut Context) -> bool {
    let enemy_intercept = match ctx.scenario.enemy_intercept() {
        Some(i) => i,
        None => return false,
    };
    let ball_loc = enemy_intercept.ball_loc.to_2d().coords;
    if (ball_loc - my_goal_center_2d()).norm() >= 3000.0 {
        return false;
    }
    let angle_car_ball = ctx.enemy().Physics.loc().to_2d().angle_to(ball_loc);
    let angle_ball_goal = ball_loc.angle_to(my_goal_center_2d());
    let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
    angle_diff < PI / 2.0
}

#[cfg(test)]
mod integration_tests {
    use behavior::runner::PUSHED;
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::{runner2::BASELINE, Runner2};

    #[test]
    fn dont_panic_when_no_intercept() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-1185.1904, 1242.3097, 133.98555),
            ball_vel: Vector3::new(-1743.6543, 1205.1072, -55.04102),
            car_loc: Vector3::new(492.75253, 567.8963, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -3.1036267, 0.0),
            car_vel: Vector3::new(-1369.871, 12.749782, 8.351),
            ..Default::default()
        });
        test.set_behavior(Runner2::soccar());
        test.sleep_millis(100);

        test.examine_eeg(|eeg| {
            assert!(!eeg
                .log
                .iter()
                .any(|x| *x == format!("{} PanicDefense", PUSHED)));
            assert!(eeg
                .log
                .iter()
                .any(|x| *x == format!("{} Offense", BASELINE)));
        });
    }

    #[test]
    fn get_boost_on_defense_if_we_have_time() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Vector3::new(-3112.23, -2548.45, 93.72),
                ball_vel: Vector3::new(-15.161, 716.11096, 37.961),
                car_loc: Vector3::new(-2500.7, -4935.0, 17.109999),
                car_rot: Rotation3::from_unreal_angles(0.0120197795, 0.056800842, -0.0034418863),
                car_vel: Vector3::new(492.951, -132.80101, 13.231),
                enemy_loc: Vector3::new(-2241.9, -2789.75, 17.02),
                enemy_rot: Rotation3::from_unreal_angles(0.009554009, 1.960959, 0.00017689279),
                enemy_vel: Vector3::new(-472.791, 1168.0409, 8.001),
                ..Default::default()
            })
            .starting_boost(0.0)
            .behavior(Runner2::soccar())
            .run();

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost < 10);

        test.sleep_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost >= 50);
    }
}
