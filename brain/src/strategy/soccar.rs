use crate::{
    behavior::{
        defense::Defense,
        higher_order::{Chain, Predicate, TryChoose, While},
        movement::{GetToFlatGround, Land, Yielder},
        offense::Offense,
        strike::{FiftyFifty, WallHit},
        taunt::{PodiumBlastoff, PodiumSpew, PodiumStare, SaltWhileDemolished, TurtleSpin},
        PreKickoff,
    },
    routing::{
        behavior::FollowRoute,
        plan::WallIntercept,
        recover::{IsSkidding, MatchIsEnded, RoundIsNotActive},
    },
    strategy::{scenario::Scenario, strategy::Strategy, Behavior, Context, Priority},
    utils::{geometry::ExtendF32, Wall},
};
use common::prelude::*;
use derive_new::new;
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;
use vec_box::vec_box;

#[derive(new)]
pub struct Soccar;

impl Strategy for Soccar {
    fn baseline(&mut self, ctx: &mut Context<'_>) -> Box<dyn Behavior> {
        // If we have no wheel contact, i.e. we're floating in the air
        if !ctx.me().OnGround {
            return Box::new(Land::new());
        }

        // If we have wheel contact, but we're not on the ground
        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            return Box::new(TryChoose::new(Priority::Idle, vec_box![
                Chain::new(Priority::Strike, vec_box![
                    FollowRoute::new(WallIntercept::new().forbid_goal_walls(true)),
                    WallHit::new(),
                ]),
                GetToFlatGround::new()
            ]));
        }

        match ctx.scenario.push_wall() {
            Wall::OwnGoal | Wall::OwnBackWall => {
                ctx.eeg.log(
                    name_of_type!(Soccar),
                    "path to ball extrapolates to back wall",
                );
                return Box::new(Defense::new());
            }
            _ => {}
        }

        if ctx.scenario.slightly_panicky_retreat()
            && ctx.scenario.possession() < Scenario::POSSESSION_CONTESTABLE
        {
            ctx.eeg
                .log(name_of_type!(Soccar), "slightly_panicky_retreat");
            return Box::new(Defense::new());
        }

        Box::new(Offense::new())
    }

    fn interrupt(
        &mut self,
        ctx: &mut Context<'_>,
        current: &dyn Behavior,
    ) -> Option<Box<dyn Behavior>> {
        if ctx.packet.GameInfo.MatchEnded {
            if current.priority() < Priority::Taunt {
                let rand = ctx.time_based_random();
                let celebrate = if rand < 0.3333333 {
                    While::new(MatchIsEnded, PodiumBlastoff::new())
                } else if rand < 0.6666667 {
                    While::new(MatchIsEnded, PodiumStare::new())
                } else {
                    While::new(MatchIsEnded, PodiumSpew::new())
                };
                return Some(Box::new(celebrate));
            }
            return None;
        }

        // Force kickoff behavior. We can't rely on the normal routing, because it
        // doesn't account for boost pads that you pick up on the way, so it dodges and
        // goes too slow.
        if current.priority() < Priority::Force && PreKickoff::is_kickoff(&ctx.packet.GameBall) {
            ctx.eeg.log(name_of_type!(Soccar), "forcing kickoff");
            return Some(Box::new(Chain::new(Priority::Force, vec![Box::new(
                PreKickoff::new(),
            )])));
        }

        if current.priority() < Priority::Strike
            && enemy_can_shoot(ctx)
            && GetToFlatGround::on_flat_ground(ctx.me())
            && !IsSkidding.evaluate(&ctx.me().into())
            && ctx.scenario.possession().abs() < Scenario::POSSESSION_CONTESTABLE
        {
            ctx.eeg.log(
                name_of_type!(Soccar),
                format!(
                    "enemy can shoot, possession = {:.2}, going for 50/50",
                    ctx.scenario.possession(),
                ),
            );
            return Some(Box::new(Chain::new(Priority::Strike, vec![Box::new(
                FiftyFifty::new(),
            )])));
        }

        if current.priority() < Priority::Save
            && ctx
                .scenario
                .impending_concede()
                .map(|b| b.t < 5.0)
                .unwrap_or(false)
            && GetToFlatGround::on_flat_ground(&ctx.me())
            && !IsSkidding.evaluate(&ctx.me().into())
        {
            ctx.eeg
                .log(name_of_type!(Soccar), "impending concede, trying to save");
            return Some(Box::new(Chain::new(Priority::Save, vec![Box::new(
                Defense::new(),
            )])));
        }

        if current.priority() < Priority::Defense
            && enemy_can_shoot(ctx)
            && GetToFlatGround::on_flat_ground(ctx.me())
            && !IsSkidding.evaluate(&ctx.me().into())
            && ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE
        {
            ctx.eeg.log(
                name_of_type!(Soccar),
                format!(
                    "enemy can shoot, possession = {:.2}, going to defense",
                    ctx.scenario.possession(),
                ),
            );
            return Some(Box::new(Chain::new(Priority::Defense, vec![Box::new(
                Defense::new(),
            )])));
        }

        if current.priority() < Priority::Defense && ctx.scenario.very_panicky_retreat() {
            ctx.eeg.log(name_of_type!(Soccar), "very_panicky_retreat");
            return Some(Box::new(Chain::new(Priority::Defense, vec![Box::new(
                Defense::new(),
            )])));
        }

        if current.priority() < Priority::Taunt && ctx.me().Demolished {
            return Some(Box::new(SaltWhileDemolished::new()));
        }
        if current.priority() < Priority::Taunt
            && UnstoppableScore.evaluate(ctx)
            && commanding_lead(ctx)
        {
            let spin = TurtleSpin::new().quick_chat_probability(0.75);
            return Some(Box::new(While::new(UnstoppableScore, spin)));
        }
        if current.priority() < Priority::Taunt && ScoringVerySoon.evaluate(ctx) {
            // Maybe do some wacky twists and stuff that might look cool.
            let spin = TurtleSpin::new();
            return Some(Box::new(While::new(ScoringVerySoon, spin)));
        }
        if current.priority() < Priority::Taunt && !ctx.packet.GameInfo.RoundActive {
            let behavior = if commanding_lead(ctx) && ball_in_enemy_half(ctx) {
                While::new(RoundIsNotActive, TurtleSpin::new())
            } else {
                While::new(
                    RoundIsNotActive,
                    Yielder::new(9999.0, Default::default()).priority(Priority::Taunt),
                )
            };
            return Some(Box::new(behavior));
        }

        None
    }
}

fn enemy_can_shoot(ctx: &mut Context<'_>) -> bool {
    let (_enemy, intercept) = match ctx.scenario.enemy_intercept() {
        Some(i) => i,
        None => return false,
    };
    let ball_loc = intercept.ball_loc.to_2d();
    let goal = ctx.game.own_goal();
    let dist_ball_to_goal = (ball_loc - goal.center_2d).norm();
    if ctx.scenario.possession() >= -Scenario::POSSESSION_CONTESTABLE {
        return false;
    }
    ctx.enemy_cars().any(|enemy| {
        let angle_car_ball = enemy
            .Physics
            .loc_2d()
            .negated_difference_and_angle_to(ball_loc);
        let angle_ball_goal = ball_loc.negated_difference_and_angle_to(goal.center_2d);
        let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
        let max_angle_diff =
            linear_interpolate(&[2500.0, 7500.0], &[PI / 2.0, PI / 4.0], dist_ball_to_goal);
        angle_diff < max_angle_diff
    })
}

fn ball_in_enemy_half(ctx: &mut Context<'_>) -> bool {
    (ctx.packet.GameBall.Physics.loc_2d() - ctx.game.enemy_goal().center_2d).norm()
        < (ctx.packet.GameBall.Physics.loc_2d() - ctx.game.own_goal().center_2d).norm()
}

struct UnstoppableScore;

impl Predicate for UnstoppableScore {
    fn name(&self) -> &str {
        name_of_type!(UnstoppableScore)
    }

    fn evaluate(&mut self, ctx: &mut Context<'_>) -> bool {
        let impending_score = some_or_else!(ctx.scenario.impending_score_conservative(), {
            return false;
        });
        let (_enemy, enemy_intercept) = some_or_else!(ctx.scenario.enemy_intercept(), {
            return true;
        });
        enemy_intercept.time >= impending_score.t + 1.0
    }
}

struct ScoringVerySoon;

impl Predicate for ScoringVerySoon {
    fn name(&self) -> &str {
        name_of_type!(ScoringVerySoon)
    }

    fn evaluate(&mut self, ctx: &mut Context<'_>) -> bool {
        let impending_score = some_or_else!(ctx.scenario.impending_score_conservative(), {
            return false;
        });
        let (_enemy, enemy_intercept) = some_or_else!(ctx.scenario.enemy_intercept(), {
            return true;
        });
        impending_score.t < 0.25 && enemy_intercept.time >= 1.0
    }
}

fn commanding_lead(ctx: &mut Context<'_>) -> bool {
    let minutes_remaining = (ctx.packet.GameInfo.GameTimeRemaining / 60.0) as i32;
    let us = ctx.packet.Teams[ctx.game.team.to_ffi() as usize].Score;
    let them = ctx.packet.Teams[ctx.game.enemy_team.to_ffi() as usize].Score;
    us - them >= minutes_remaining
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        eeg::Event,
        integration_tests::{TestRunner, TestScenario},
    };
    use brain_test_data::recordings;
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    fn dont_panic_when_no_intercept() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1185.1904, 1242.3097, 133.98555),
                ball_vel: Vector3::new(-1743.6543, 1205.1072, -55.04102),
                car_loc: Point3::new(492.75253, 567.8963, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -3.1036267, 0.0),
                car_vel: Vector3::new(-1369.871, 12.749782, 8.351),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(100);

        test.examine_events(|events| {
            assert!(!events.contains(&Event::PanicDefense));
            assert!(events.contains(&Event::Offense));
        });
    }

    #[test]
    fn get_boost_on_defense_if_we_have_time() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-3112.23, -2548.45, 93.72),
                ball_vel: Vector3::new(-15.161, 716.11096, 37.961),
                car_loc: Point3::new(-2500.7, -4935.0, 17.109999),
                car_rot: Rotation3::from_unreal_angles(0.0120197795, 0.056800842, -0.0034418863),
                car_vel: Vector3::new(492.951, -132.80101, 13.231),
                enemy_loc: Point3::new(-2241.9, -2789.75, 17.02),
                enemy_rot: Rotation3::from_unreal_angles(0.009554009, 1.960959, 0.00017689279),
                enemy_vel: Vector3::new(-472.791, 1168.0409, 8.001),
                ..Default::default()
            })
            .starting_boost(0.0)
            .soccar()
            .run();

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost < 10);

        test.sleep_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost >= 50);
    }

    #[test]
    #[ignore(note = "TODO")]
    fn get_boost_on_offense_if_we_have_time() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(810.51, 373.18, 227.86),
                ball_vel: Vector3::new(1538.241, 861.4509, -272.631),
                car_loc: Point3::new(837.23, -759.61, 18.4),
                car_rot: Rotation3::from_unreal_angles(-0.08508434, -0.4163043, 0.009112751),
                car_vel: Vector3::new(1678.101, -205.961, -160.641),
                ..Default::default()
            })
            .starting_boost(28.0)
            .soccar()
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost >= 80);
        // Go to the closest boost, don't go off to some distant corner.
        assert!(packet.GameCars[0].Physics.loc().y.abs() < 2000.0);
    }

    #[test]
    fn clear_stationary_ball() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-500.0, -5000.0, 0.0),
                car_loc: Point3::new(500.0, -5000.0, 17.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 210_f32.to_radians(), 0.0),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.Location.X < -1000.0);
    }

    #[test]
    fn clear_defensive_ball() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::CLEAR_DEFENSIVE_BALL, 53.0)
            .soccar()
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.Location.X >= 1500.0);
    }

    #[test]
    #[ignore(note = "TODO")]
    fn dont_allow_long_shot() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::DONT_ALLOW_LONG_SHOT, 282.5)
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(7000);

        assert!(!test.enemy_has_scored());
    }

    #[test]
    fn transition_from_defense_to_save() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::TRANSITION_FROM_DEFENSE_TO_SAVE, 109.0)
            .starting_boost(30.0)
            .soccar()
            .run_for_millis(7000);

        assert!(!test.enemy_has_scored());
    }
}
