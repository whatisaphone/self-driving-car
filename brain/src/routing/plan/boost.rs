use common::prelude::*;
use nalgebra::Point2;
use routing::{
    models::{CarState, RoutePlan, RoutePlanError, RoutePlanner},
    plan::ground_powerslide::GroundPowerslideTurn,
    recover::{IsSkidding, NotOnFlatGround},
};
use std::f32::consts::PI;
use strategy::Scenario;

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

        let target_loc = Point2::new(-3072.0, -4096.0); // TODO: not hardcode

        guard!(
            start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: target_loc,
            },
        );

        if (target_loc - start.loc.to_2d()).norm() < 500.0
            && start.vel.to_2d().norm() < 100.0
            && start
                .forward_axis_2d()
                .rotation_to(&(target_loc - start.loc.to_2d()).to_axis())
                .angle()
                .abs()
                >= PI / 3.0
        {
            return Err(RoutePlanError::OtherError("TODO: easier to flip to pad"));
        }

        GroundPowerslideTurn::new(target_loc, self.then_face, None)
            .plan(start_time, start, scenario)
    }
}
