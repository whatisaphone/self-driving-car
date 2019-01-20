use crate::{
    behavior::{
        higher_order::{Predicate, TimeLimit, TryChoose, While},
        movement::{DriveTowards, GetToFlatGround, QuickJumpAndDodge, SkidRecover},
        offense::ResetBehindBall,
    },
    routing::{
        behavior::FollowRoute,
        models::{CarState, RoutePlanError},
        plan::GroundDrive,
        StraightMode,
    },
    strategy::{Behavior, Context, Priority, Scenario},
};
use common::{physics::car_forward_axis, prelude::*};
use derive_new::new;
use nalgebra::Point2;
use nameof::name_of_type;
use std::f32::consts::PI;
use vec_box::vec_box;

const SKIDDING_THRESHOLD: f32 = 0.95;

impl RoutePlanError {
    pub fn recover(&self, ctx: &mut Context<'_>) -> Option<Box<dyn Behavior>> {
        match *self {
            RoutePlanError::MustBeOnFlatGround => Some(Box::new(While::new(
                NotOnFlatGround,
                GetToFlatGround::new(),
            ))),
            RoutePlanError::MustNotBeSkidding { recover_target_loc } => {
                Some(Box::new(SkidRecover::new(recover_target_loc)))
            }
            RoutePlanError::UnknownIntercept => {
                let target_loc = ctx.scenario.ball_prediction().last().loc;
                let wander = DriveTowards::new(target_loc.to_2d());
                Some(Box::new(TimeLimit::new(1.0, wander)))
            }
            RoutePlanError::TurningRadiusTooTight => {
                if let Some(b) = check_easy_flip_recover(ctx) {
                    return Some(b);
                }

                let ball_loc = ctx.scenario.ball_prediction().at_time_or_last(2.5).loc;
                Some(Box::new(TryChoose::new(Priority::Idle, vec_box![
                    FollowRoute::new(
                        GroundDrive::new(ball_loc.to_2d()).straight_mode(StraightMode::Fake)
                    )
                    .never_recover(true),
                    ResetBehindBall::behind_loc(ball_loc.to_2d(), 1500.0).never_recover(true),
                ])))
            }
            RoutePlanError::MustBeFacingTarget => {
                if ctx.me().Physics.vel_2d().norm() < 400.0
                    && ctx.packet.GameBall.Physics.vel_2d().norm() < 400.0
                {
                    ctx.eeg
                        .log(stringify!(recover), "we gotta get things moving!");
                    let ball_loc = ctx.scenario.ball_prediction().at_time_or_last(2.5).loc;
                    return Some(Box::new(
                        ResetBehindBall::behind_loc(ball_loc.to_2d(), 1000.0).never_recover(true),
                    ));
                }
                None
            }
            RoutePlanError::MovingTooFast
            | RoutePlanError::CannotOperateWall
            | RoutePlanError::NoWallIntercept
            | RoutePlanError::TurnAngleTooLarge
            | RoutePlanError::OtherError(_) => None,
        }
    }
}

/// Check if the ball is roughly in front of us and we can easily just smack it
/// for free.
fn check_easy_flip_recover(ctx: &mut Context<'_>) -> Option<Box<dyn Behavior>> {
    let ball = ctx.scenario.ball_prediction().at_time_or_last(0.5);
    let me_loc = ctx.me().Physics.loc_2d();
    let me_forward = ctx.me().Physics.forward_axis_2d();

    let angle_to_ball = me_forward.angle_to(&(ball.loc.to_2d() - me_loc).to_axis());
    let own_goal_loc = ctx.game.own_goal().center_2d;
    let angle_to_own_goal = me_forward.angle_to(&(own_goal_loc - me_loc).to_axis());
    if (me_loc - ball.loc.to_2d()).norm() < 350.0
        && ball.loc.z < 120.0
        && ball.vel.z.abs() < 50.0
        && angle_to_ball.abs() < PI / 3.0
        && angle_to_own_goal.abs() >= PI / 3.0
    {
        ctx.eeg.log(
            stringify!(recover),
            "the ball is right here, I can't resist!",
        );
        let dodge = me_forward.angle_to(&(ball.loc.to_2d() - ctx.me().Physics.loc_2d()).to_axis());
        return Some(Box::new(QuickJumpAndDodge::new().angle(dodge)));
    }

    None
}

pub struct NotOnFlatGround;

impl NotOnFlatGround {
    pub fn evaluate(&mut self, state: &CarState) -> bool {
        let rot = state.forward_axis();
        let rot_flat = rot.to_2d().to_3d();
        rot.angle(&rot_flat) >= 15.0_f32.to_radians()
    }
}

impl Predicate for NotOnFlatGround {
    fn name(&self) -> &str {
        name_of_type!(NotOnFlatGround)
    }

    fn evaluate(&mut self, ctx: &'_ mut Context<'_>) -> bool {
        !GetToFlatGround::on_flat_ground(ctx.me())
    }
}

pub struct IsSkidding;

impl IsSkidding {
    pub fn evaluate(&mut self, state: &CarState) -> bool {
        state.vel.norm() >= 100.0
            && state.vel.normalize().dot(&car_forward_axis(state.rot)) < SKIDDING_THRESHOLD
    }
}

#[derive(new)]
pub struct NotFacingTarget2D {
    target_loc: Point2<f32>,
}

impl NotFacingTarget2D {
    pub fn evaluate(&self, state: &CarState) -> bool {
        let forward = state.forward_axis_2d();
        let to_target = (self.target_loc - state.loc.to_2d()).to_axis();
        forward.angle_to(&to_target).abs() >= PI / 6.0
    }
}

impl Predicate for IsSkidding {
    fn name(&self) -> &str {
        name_of_type!(IsSkidding)
    }

    fn evaluate(&mut self, ctx: &mut Context<'_>) -> bool {
        self.evaluate(&ctx.me().into())
    }
}

pub struct WeDontWinTheRace;

impl Predicate for WeDontWinTheRace {
    fn name(&self) -> &str {
        name_of_type!(WeDontWinTheRace)
    }

    fn evaluate(&mut self, ctx: &mut Context<'_>) -> bool {
        ctx.scenario.possession() < Scenario::POSSESSION_CONTESTABLE
    }
}
