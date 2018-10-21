use behavior::{Behavior, NullBehavior, Predicate, While};
use common::ext::ExtendPhysics;
use maneuvers::GetToFlatGround;
use routing::plan::RoutePlanError;
use strategy::Context;

const SKIDDING_THRESHOLD: f32 = 0.95;

impl RoutePlanError {
    pub fn recover(&self) -> Option<Box<Behavior>> {
        match *self {
            RoutePlanError::MustBeOnFlatGround => Some(Box::new(While::new(
                NotOnFlatGround,
                GetToFlatGround::new(),
            ))),
            RoutePlanError::MustNotBeSkidding => {
                Some(Box::new(While::new(IsSkidding, NullBehavior::new())))
            }
            _ => None,
        }
    }
}

pub struct NotOnFlatGround;

impl Predicate for NotOnFlatGround {
    fn name(&self) -> &str {
        stringify!(NotOnFlatGround)
    }

    fn evaluate(&mut self, ctx: &'_ mut Context) -> bool {
        !GetToFlatGround::on_flat_ground(ctx.packet)
    }
}

pub struct IsSkidding;

impl Predicate for IsSkidding {
    fn name(&self) -> &str {
        stringify!(IsSkidding)
    }

    fn evaluate(&mut self, ctx: &mut Context) -> bool {
        let me = ctx.me();
        if me.Physics.vel().norm() >= 10.0 {
            if me.Physics.vel().normalize().dot(&me.Physics.forward_axis()) < SKIDDING_THRESHOLD {
                return true;
            }
        }
        false
    }
}
