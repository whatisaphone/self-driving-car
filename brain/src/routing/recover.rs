use crate::{
    behavior::{
        offense2::reset_behind_ball::ResetBehindBall, Behavior, Predicate, TimeLimit, While,
    },
    maneuvers::{DriveTowards, GetToFlatGround},
    mechanics::{QuickJumpAndDodge, SkidRecover},
    routing::models::{CarState, RoutePlanError},
    strategy::{Context, Scenario},
};
use common::{physics::car_forward_axis, prelude::*};
use derive_new::new;
use nalgebra::Point2;
use std::f32::consts::PI;

const SKIDDING_THRESHOLD: f32 = 0.95;

impl RoutePlanError {
    pub fn recover(&self, ctx: &mut Context) -> Option<Box<Behavior>> {
        match *self {
            RoutePlanError::MustBeOnFlatGround => Some(Box::new(While::new(
                NotOnFlatGround,
                GetToFlatGround::new(),
            ))),
            RoutePlanError::MustNotBeSkidding { recover_target_loc } => {
                if let Some(b) = check_easy_flip_recover(ctx) {
                    return Some(b);
                }

                Some(Box::new(While::new(
                    IsSkidding,
                    SkidRecover::new(recover_target_loc),
                )))
            }
            RoutePlanError::UnknownIntercept => {
                let target_loc = ctx.scenario.ball_prediction().iter().last().unwrap().loc;
                let wander = DriveTowards::new(target_loc.to_2d());
                Some(Box::new(TimeLimit::new(1.0, wander)))
            }
            RoutePlanError::TurningRadiusTooTight => {
                if let Some(b) = check_easy_flip_recover(ctx) {
                    return Some(b);
                }

                let ball_loc = ctx.scenario.ball_prediction().at_time(2.5).unwrap().loc;
                Some(Box::new(
                    ResetBehindBall::behind_loc(ball_loc.to_2d()).never_recover(true),
                ))
            }
            RoutePlanError::MustBeFacingTarget => {
                if ctx.me().Physics.vel_2d().norm() < 400.0
                    && ctx.packet.GameBall.Physics.vel_2d().norm() < 400.0
                {
                    ctx.eeg
                        .log("[MustBeFacingTarget] we gotta get things moving!");
                    let ball_loc = ctx.scenario.ball_prediction().at_time(2.5).unwrap().loc;
                    return Some(Box::new(
                        ResetBehindBall::behind_loc(ball_loc.to_2d()).never_recover(true),
                    ));
                }
                None
            }
            RoutePlanError::MovingTooFast | RoutePlanError::OtherError(_) => None,
        }
    }
}

/// Check if the ball is roughly in front of us and we can easily just smack it
/// for free.
fn check_easy_flip_recover(ctx: &mut Context) -> Option<Box<Behavior>> {
    let ball = ctx.scenario.ball_prediction().at_time(0.5).unwrap();
    let me_loc = ctx.me().Physics.loc_2d();
    let me_forward = ctx.me().Physics.forward_axis_2d();

    let me_rot_to_ball = me_forward.rotation_to(&(ball.loc.to_2d() - me_loc).to_axis());
    let own_goal_loc = ctx.game.own_goal().center_2d;
    let me_rot_to_own_goal = me_forward.rotation_to(&(own_goal_loc - me_loc).to_axis());
    if (me_loc - ball.loc.to_2d()).norm() < 400.0
        && ball.loc.z < 120.0
        && ball.vel.z.abs() < 50.0
        && me_rot_to_ball.angle().abs() < PI / 3.0
        && me_rot_to_own_goal.angle().abs() >= PI / 3.0
    {
        ctx.eeg
            .log("[check_easy_flip_recover] the ball is right here, I can't resist!");
        let dodge =
            me_forward.rotation_to(&(ball.loc.to_2d() - ctx.me().Physics.loc_2d()).to_axis());
        return Some(Box::new(QuickJumpAndDodge::new().angle(dodge.angle())));
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
        stringify!(NotOnFlatGround)
    }

    fn evaluate(&mut self, ctx: &'_ mut Context) -> bool {
        !GetToFlatGround::on_flat_ground(ctx.me())
    }
}

pub struct IsSkidding;

impl IsSkidding {
    pub fn evaluate(&mut self, state: &CarState) -> bool {
        if state.vel.norm() >= 100.0 {
            if state.vel.normalize().dot(&car_forward_axis(state.rot)) < SKIDDING_THRESHOLD {
                return true;
            }
        }
        false
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
        forward.rotation_to(&to_target).angle().abs() >= PI / 6.0
    }
}

impl Predicate for IsSkidding {
    fn name(&self) -> &str {
        stringify!(IsSkidding)
    }

    fn evaluate(&mut self, ctx: &mut Context) -> bool {
        let me = ctx.me();
        if me.Physics.vel().norm() >= 100.0 {
            if me.Physics.vel().normalize().dot(&me.Physics.forward_axis()) < SKIDDING_THRESHOLD {
                return true;
            }
        }
        false
    }
}

pub struct WeDontWinTheRace;

impl Predicate for WeDontWinTheRace {
    fn name(&self) -> &str {
        stringify!(WeDontWinTheRace)
    }

    fn evaluate(&mut self, ctx: &mut Context) -> bool {
        ctx.scenario.possession() < Scenario::POSSESSION_CONTESTABLE
    }
}
