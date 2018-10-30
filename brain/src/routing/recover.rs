use behavior::{Behavior, NullBehavior, Predicate, TimeLimit, While};
use common::{physics::car_forward_axis, prelude::*};
use maneuvers::{DriveTowards, GetToFlatGround};
use routing::models::{CarState, RoutePlanError};
use strategy::Context;
use utils::geometry::ExtendPoint3;

const SKIDDING_THRESHOLD: f32 = 0.95;

impl RoutePlanError {
    pub fn recover(&self, ctx: &mut Context) -> Option<Box<Behavior>> {
        match *self {
            RoutePlanError::MustBeOnFlatGround => Some(Box::new(While::new(
                NotOnFlatGround,
                GetToFlatGround::new(),
            ))),
            RoutePlanError::MustNotBeSkidding => {
                Some(Box::new(While::new(IsSkidding, NullBehavior::new())))
            }
            RoutePlanError::UnknownIntercept => {
                let target_loc = ctx.scenario.ball_prediction().iter().last().unwrap().loc;
                let wander = DriveTowards::new(target_loc.to_2d());
                Some(Box::new(TimeLimit::new(1.0, wander)))
            }
            RoutePlanError::OtherError(_) => None,
        }
    }
}

pub struct NotOnFlatGround;

impl NotOnFlatGround {
    pub fn evaluate(&mut self, state: &CarState) -> bool {
        let rot = state.forward_axis();
        let rot_flat = state.forward_axis().to_2d().to_3d();
        rot.angle(&rot_flat) >= 15.0_f32.to_radians()
    }
}

impl Predicate for NotOnFlatGround {
    fn name(&self) -> &str {
        stringify!(NotOnFlatGround)
    }

    fn evaluate(&mut self, ctx: &'_ mut Context) -> bool {
        !GetToFlatGround::on_flat_ground(ctx.packet)
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
