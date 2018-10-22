use behavior::{Behavior, NullBehavior, Predicate, TimeLimit, While};
use common::ext::ExtendPhysics;
use maneuvers::{DriveTowards, GetToFlatGround};
use nalgebra::Point3;
use routing::plan::RoutePlanError;
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
        if me.Physics.vel().norm() >= 100.0 {
            if me.Physics.vel().normalize().dot(&me.Physics.forward_axis()) < SKIDDING_THRESHOLD {
                return true;
            }
        }
        false
    }
}

pub struct FutureBallLoc {
    time: f32,
    loc: Point3<f32>,
}

impl FutureBallLoc {
    pub fn new(time: f32, loc: Point3<f32>) -> Self {
        Self { time, loc }
    }
}

impl Predicate for FutureBallLoc {
    fn name(&self) -> &str {
        stringify!(FutureBallLoc)
    }

    fn evaluate(&mut self, ctx: &mut Context) -> bool {
        let t = self.time - ctx.packet.GameInfo.TimeSeconds;
        let expected = some_or_else!(ctx.scenario.ball_prediction().at_time(t), {
            return false;
        });
        let error = (self.loc - expected.loc).norm();
        error < 10.0
    }
}
