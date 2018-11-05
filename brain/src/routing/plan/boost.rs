use common::prelude::*;
use nalgebra::{center, Point2};
use ordered_float::NotNan;
use routing::{
    models::{PlanningContext, RoutePlan, RoutePlanError, RoutePlanner},
    plan::ground_powerslide::GroundPowerslideTurn,
    recover::{IsSkidding, NotOnFlatGround},
};
use std::f32::consts::PI;
use strategy::BoostPickup;

#[derive(Clone)]
pub struct GetDollar {
    then_face: Point2<f32>,
}

impl GetDollar {
    #[allow(dead_code)]
    pub fn then_face(then_face: Point2<f32>) -> Self {
        Self { then_face }
    }
}

impl RoutePlanner for GetDollar {
    fn name(&self) -> &'static str {
        stringify!(GetDollar)
    }

    fn plan(&self, ctx: &PlanningContext) -> Result<RoutePlan, RoutePlanError> {
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );

        let pickup = match self.chooose_pickup(ctx) {
            Some(p) => p,
            None => return Err(RoutePlanError::OtherError("no pickup found")),
        };

        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: pickup.loc,
            },
        );

        if (pickup.loc - ctx.start.loc.to_2d()).norm() < 500.0
            && ctx.start.vel.to_2d().norm() < 100.0
            && ctx
                .start
                .forward_axis_2d()
                .rotation_to(&(pickup.loc - ctx.start.loc.to_2d()).to_axis())
                .angle()
                .abs()
                >= PI / 3.0
        {
            return Err(RoutePlanError::OtherError("TODO: easier to flip to pad"));
        }

        GroundPowerslideTurn::new(pickup.loc, self.then_face, None).plan(ctx)
    }
}

impl GetDollar {
    fn chooose_pickup<'a>(&self, ctx: &'a PlanningContext) -> Option<&'a BoostPickup> {
        ctx.game.boost_dollars().iter().min_by_key(|pickup| {
            let car_adjusted_loc = ctx.start.loc.to_2d()
                + ctx.start.forward_axis_2d().as_ref() * 500.0
                + ctx.start.vel.to_2d() * 0.5;

            let ball = ctx.ball_prediction.start();
            let ball_adjusted_loc = ball.loc.to_2d() + ball.vel.to_2d() * 2.0;

            let eval_loc = center(&car_adjusted_loc, &ball_adjusted_loc);
            let score = (pickup.loc - eval_loc).norm();
            NotNan::new(score).unwrap()
        })
    }
}
