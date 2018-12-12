use crate::{
    behavior::{Behavior, Predicate, TimeLimit, While},
    maneuvers::{DriveTowards, GetToFlatGround},
    mechanics::SkidRecover,
    routing::{
        behavior::FollowRoute,
        models::{CarState, RoutePlanError},
        plan::GroundDrive,
    },
    strategy::Context,
};
use common::{physics::car_forward_axis, prelude::*};
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
            RoutePlanError::MustNotBeSkidding { recover_target_loc } => Some(Box::new(While::new(
                IsSkidding,
                SkidRecover::new(recover_target_loc),
            ))),
            RoutePlanError::UnknownIntercept => {
                let target_loc = ctx.scenario.ball_prediction().iter().last().unwrap().loc;
                let wander = DriveTowards::new(target_loc.to_2d());
                Some(Box::new(TimeLimit::new(1.0, wander)))
            }
            RoutePlanError::TurningRadiusTooTight => {
                ctx.eeg.log("going behind the ball to try again");
                let ball_loc = ctx.scenario.ball_prediction().at_time(2.5).unwrap().loc;
                let behind_ball = Point2::new(
                    ball_loc.x,
                    ball_loc.y + ctx.game.own_goal().center_2d.y.signum() * 1500.0,
                );
                // TODO: make sure we're not trying to leave the field?
                let straight = GroundDrive::new(behind_ball);
                return Some(Box::new(FollowRoute::new(straight).never_recover()));
            }
            RoutePlanError::MustBeFacingTarget
            | RoutePlanError::MovingTooFast
            | RoutePlanError::OtherError(_) => None,
        }
    }
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
