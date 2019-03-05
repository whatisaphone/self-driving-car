use crate::{
    behavior::{
        defense::{retreating_save::RetreatingSave, HitToOwnCorner},
        higher_order::TryChoose,
        strike::GroundedHit,
    },
    eeg::{color, Drawable},
    helpers::intercept::naive_ground_intercept_2,
    strategy::{Action, Behavior, Context, Goal, Priority, Scenario},
    utils::geometry::ExtendF32,
};
use common::{prelude::*, Time};
use nalgebra::{Point2, Point3};
use nameof::name_of_type;
use ordered_float::NotNan;
use simulate::linear_interpolate;
use std::f32::consts::PI;

/// Push the ball to our own corner if it makes sense given the situation;
/// otherwise abort.
pub struct PushToOwnCorner;

impl PushToOwnCorner {
    const MAX_BALL_Z: f32 = HitToOwnCorner::MAX_BALL_Z;

    pub fn new() -> Self {
        Self
    }

    fn shot_angle(ball_loc: Point3<f32>, car_loc: Point3<f32>, aim_loc: Point2<f32>) -> f32 {
        let angle_me_ball = car_loc
            .to_2d()
            .negated_difference_and_angle_to(ball_loc.to_2d());
        let angle_ball_goal = ball_loc.to_2d().negated_difference_and_angle_to(aim_loc);
        (angle_me_ball - angle_ball_goal).normalize_angle().abs()
    }

    fn goal_angle(ball_loc: Point3<f32>, goal: &Goal) -> f32 {
        let goal_to_ball_axis = (ball_loc.to_2d() - goal.center_2d).to_axis();
        goal_to_ball_axis.angle_to(&goal.normal_2d).abs()
    }
}

impl Behavior for PushToOwnCorner {
    fn name(&self) -> &str {
        name_of_type!(PushToOwnCorner)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let impending_concede_soon = ctx
            .scenario
            .impending_concede()
            .or_else(|| RetreatingSave::impending_dangerous_ball(ctx))
            .map(|f| f.t < 5.0)
            .unwrap_or_default();

        if impending_concede_soon {
            return Action::tail_call(hit_to_safety(ctx));
        }

        let me_intercept =
            naive_ground_intercept_2(&ctx.me().into(), ctx.scenario.ball_prediction(), |ball| {
                ball.loc.z < Self::MAX_BALL_Z
            });

        // If the ball is moving quickly towards our half, assume the enemy can't change
        // its angle that much.
        let own_goal = ctx.game.own_goal();
        let danger_angle = linear_interpolate(
            &[1000.0, 1500.0],
            &[PI / 2.0, PI / 4.0],
            ctx.packet
                .GameBall
                .Physics
                .vel_2d()
                .dot(&-own_goal.normal_2d),
        );

        let enemy_shootable_intercept = ctx
            .enemy_cars()
            .filter_map(|enemy| {
                naive_ground_intercept_2(&enemy.into(), ctx.scenario.ball_prediction(), |ball| {
                    ball.loc.z < GroundedHit::MAX_BALL_Z
                        && Self::shot_angle(ball.loc, enemy.Physics.loc(), own_goal.center_2d)
                            < danger_angle
                        && Self::goal_angle(ball.loc, own_goal) < PI / 3.0
                })
            })
            .min_by_key(|i| NotNan::new(i.time).unwrap());

        if let Some(ref i) = me_intercept {
            ctx.eeg
                .log_pretty(self.name(), "me_intercept", Time(i.time));
            ctx.eeg.draw(Drawable::GhostBall(
                i.ball_loc,
                color::for_team(ctx.game.team),
            ));
        }
        if let Some(ref i) = enemy_shootable_intercept {
            ctx.eeg
                .log_pretty(self.name(), "enemy_shoot_intercept", Time(i.time));
            ctx.eeg.draw(Drawable::GhostBall(
                i.ball_loc,
                color::for_team(ctx.game.enemy_team),
            ));
        }

        match (me_intercept, enemy_shootable_intercept) {
            (_, None) => {
                ctx.eeg.log(self.name(), "safe for now");
                Action::Return
            }
            (None, _) => {
                ctx.eeg.log(self.name(), "can't reach ball");
                Action::Abort
            }
            (Some(me_intercept), Some(_enemy_intercept)) => {
                if ctx.scenario.possession() >= 3.0 {
                    ctx.eeg
                        .log(self.name(), "we have all the time in the world");
                    return Action::Abort;
                } else if ctx.scenario.possession() >= Scenario::POSSESSION_CONTESTABLE {
                    ctx.eeg.log(self.name(), "swatting ball away from enemy");
                    return Action::tail_call(hit_to_safety(ctx));
                } else if ctx.scenario.possession() >= -Scenario::POSSESSION_CONTESTABLE {
                    ctx.eeg.log(self.name(), "defensive race");
                    return Action::tail_call(hit_to_safety(ctx));
                }

                // Things are looking bad. No possession and we're behind the ball. What are our
                // options?
                ctx.eeg.log(self.name(), "things are looking dire");

                let retreat_angle = (me_intercept.ball_loc.to_2d() - ctx.me().Physics.loc_2d())
                    .angle_to(&(ctx.game.own_goal().center_2d - me_intercept.ball_loc.to_2d()));
                if retreat_angle.abs() < PI / 6.0 {
                    ctx.eeg.log(self.name(), "the ball is on the way back");
                    return Action::tail_call(hit_to_safety(ctx));
                }

                if (ctx.game.own_goal().center_2d - me_intercept.ball_loc.to_2d()).norm() < 1000.0 {
                    ctx.eeg
                        .log(self.name(), "the ball will be right by the goal");
                    return Action::tail_call(hit_to_safety(ctx));
                }

                ctx.eeg.log(self.name(), "all hope is lost");
                // Most likely RetreatingSave will Abort and we'll bubble back up to
                // PanicDefense, but sometimes we can actually make a save.
                Action::tail_call(RetreatingSave::new())
            }
        }
    }
}

fn hit_to_safety(ctx: &mut Context<'_>) -> impl Behavior {
    let goal_loc = ctx.game.own_goal().center_2d;
    let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
    let me_loc = ctx.me().Physics.loc_2d();

    let axis = (me_loc - goal_loc).to_axis();
    let ball_dist = (ball_loc - goal_loc).dot(&axis);
    let me_dist = (me_loc - goal_loc).dot(&axis);
    let goalside = ball_dist - me_dist;

    let mut choices = Vec::<Box<dyn Behavior>>::new();
    if goalside >= 0.0 {
        choices.push(Box::new(RetreatingSave::new()));
    } else {
        choices.push(Box::new(HitToOwnCorner::new()));
    }
    TryChoose::new(Priority::Idle, choices)
}

#[cfg(test)]
mod integration_tests {
    use crate::{eeg::Event, integration_tests::TestRunner};
    use brain_test_data::recordings;
    use common::{prelude::*, rl};
    use nalgebra::Point2;

    #[test]
    fn let_the_ball_enter_our_corner() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::LET_THE_BALL_ENTER_OUR_CORNER, 133.0)
            .soccar()
            .run_for_millis(6000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let own_goal = Point2::new(0.0, -rl::FIELD_MAX_Y);
        let goal_to_ball_dist = (packet.GameBall.Physics.loc_2d() - own_goal).norm();
        assert!(goal_to_ball_dist >= 2000.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::PanicDefense));
            assert!(!events.contains(&Event::HitToOwnCorner));
        });
    }

    #[test]
    fn save_ball_rolling_towards_box() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::SAVE_BALL_ROLLING_TOWARDS_BOX, 160.0)
            .soccar()
            .run_for_millis(3000);

        assert!(!test.enemy_has_scored());

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        println!("ball_loc = {:?}", ball_loc);
        assert!(ball_loc.y < 4500.0);

        test.examine_events(|events| {
            assert!(events.contains(&Event::HitToOwnCorner));
            assert!(!events.contains(&Event::PanicDefense));
        });
    }
}
