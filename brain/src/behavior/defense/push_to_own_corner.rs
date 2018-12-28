use crate::{
    behavior::{defense::HitToOwnCorner, strike::GroundedHit},
    eeg::{color, Drawable},
    predict::naive_ground_intercept_2,
    strategy::{Action, Behavior, Context, Goal, Scenario},
    utils::geometry::ExtendF32,
};
use common::{prelude::*, Time};
use nalgebra::{Point2, Point3};
use nameof::name_of_type;
use ordered_float::NotNan;
use std::f32::consts::PI;

/// Push the ball to our own corner if it makes sense given the situation;
/// otherwise abort.
pub struct PushToOwnCorner;

impl PushToOwnCorner {
    const MAX_BALL_Z: f32 = HitToOwnCorner::MAX_BALL_Z;

    pub fn new() -> Self {
        PushToOwnCorner
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
        goal_to_ball_axis.rotation_to(&goal.normal_2d).angle().abs()
    }
}

impl Behavior for PushToOwnCorner {
    fn name(&self) -> &str {
        name_of_type!(PushToOwnCorner)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        let impending_concede_soon = ctx
            .scenario
            .impending_concede()
            .map(|f| f.t < 5.0)
            .unwrap_or_default();

        let me_intercept =
            naive_ground_intercept_2(&ctx.me().into(), ctx.scenario.ball_prediction(), |ball| {
                ball.loc.z < Self::MAX_BALL_Z
            });

        let enemy_shootable_intercept = ctx
            .enemy_cars()
            .filter_map(|enemy| {
                naive_ground_intercept_2(&enemy.into(), ctx.scenario.ball_prediction(), |ball| {
                    let own_goal = ctx.game.own_goal().center_2d;
                    ball.loc.z < GroundedHit::MAX_BALL_Z
                        && Self::shot_angle(ball.loc, enemy.Physics.loc(), own_goal) < PI / 3.0
                        && Self::goal_angle(ball.loc, ctx.game.own_goal()) < PI / 3.0
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
                if !impending_concede_soon {
                    ctx.eeg.log(self.name(), "safe for now");
                    Action::Return
                } else {
                    ctx.eeg.log(self.name(), "hitting away from goal");
                    Action::call(HitToOwnCorner::new())
                }
            }
            (None, _) => {
                ctx.eeg.log(self.name(), "can't reach ball");
                Action::Abort
            }
            (Some(_), Some(_)) => {
                if ctx.scenario.possession() >= 3.0 {
                    ctx.eeg
                        .log(self.name(), "we have all the time in the world");
                    Action::Abort
                } else if ctx.scenario.possession() >= Scenario::POSSESSION_CONTESTABLE {
                    ctx.eeg.log(self.name(), "swatting ball away from enemy");
                    Action::call(HitToOwnCorner::new())
                } else if ctx.scenario.possession() >= -Scenario::POSSESSION_CONTESTABLE {
                    ctx.eeg.log(self.name(), "defensive race");
                    Action::call(HitToOwnCorner::new())
                } else {
                    ctx.eeg.log(self.name(), "can't reach ball before enemy");
                    Action::Abort
                }
            }
        }
    }
}
