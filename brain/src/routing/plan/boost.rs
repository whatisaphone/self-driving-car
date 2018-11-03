use common::prelude::*;
use nalgebra::{center, Point2};
use ordered_float::NotNan;
use routing::{
    models::{CarState, RoutePlan, RoutePlanError, RoutePlanner},
    plan::ground_powerslide::GroundPowerslideTurn,
    recover::{IsSkidding, NotOnFlatGround},
};
use std::f32::consts::PI;
use strategy::{BoostPickup, Scenario};

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
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);

        let pickup = match self.chooose_pickup(start, scenario) {
            Some(p) => p,
            None => return Err(RoutePlanError::OtherError("no pickup found")),
        };

        guard!(
            start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: pickup.loc,
            },
        );

        if (pickup.loc - start.loc.to_2d()).norm() < 500.0
            && start.vel.to_2d().norm() < 100.0
            && start
                .forward_axis_2d()
                .rotation_to(&(pickup.loc - start.loc.to_2d()).to_axis())
                .angle()
                .abs()
                >= PI / 3.0
        {
            return Err(RoutePlanError::OtherError("TODO: easier to flip to pad"));
        }

        GroundPowerslideTurn::new(pickup.loc, self.then_face, None)
            .plan(start_time, start, scenario)
    }
}

impl GetDollar {
    fn chooose_pickup<'a>(
        &self,
        start: &CarState,
        scenario: &'a Scenario,
    ) -> Option<&'a BoostPickup> {
        scenario.game.boost_dollars().iter().min_by_key(|pickup| {
            let car_adjusted_loc = start.loc.to_2d()
                + start.forward_axis_2d().as_ref() * 500.0
                + start.vel.to_2d() * 0.5;
            let ball_loc = scenario.ball_prediction().start().loc.to_2d();
            let eval_loc = center(&car_adjusted_loc, &ball_loc);
            let score = (pickup.loc - eval_loc).norm();
            NotNan::new(score).unwrap()
        })
    }
}
