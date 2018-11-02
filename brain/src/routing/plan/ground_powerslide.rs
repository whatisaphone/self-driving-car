use common::prelude::*;
use nalgebra::Point2;
use routing::{
    models::{CarState, RoutePlan, RoutePlanError, RoutePlanner},
    plan::{
        ground_straight::GroundStraightPlanner,
        ground_turn::TurnPlanner,
        higher_order::{ChainedPlanner, StaticPlanner},
    },
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{PowerslideTurn, StraightMode},
};
use simulate::CarPowerslideTurn;
use strategy::Scenario;

#[derive(Clone, new)]
pub struct GroundPowerslideTurn {
    target_loc: Point2<f32>,
    target_face: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for GroundPowerslideTurn {
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        TurnPlanner::new(
            self.target_loc,
            Some(Box::new(GroundPowerslideEssence::new(
                self.target_loc,
                self.target_face,
                self.next.clone(),
            ))),
        )
        .plan(start_time, start, scenario)
    }
}

/// This is the part after the car is facing the right way.
#[derive(Clone, new)]
struct GroundPowerslideEssence {
    target_loc: Point2<f32>,
    target_face: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for GroundPowerslideEssence {
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);
        guard!(
            start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        // First pass to estimate approach speed
        let asap = 0.0; // TODO: teach GroundStraight about not caring about the exact time of arrival
        let end_chop = 2.0; // Leave some time for the actual slide.
        let first_pass =
            GroundStraightPlanner::new(self.target_loc, asap, end_chop, StraightMode::Real)
                .plan(start_time, start, scenario)?;

        let slide_start = first_pass.segment.start();
        let blueprint = CarPowerslideTurn::evaluate(
            slide_start.loc.to_2d(),
            slide_start.vel.to_2d(),
            1.0,
            (self.target_face - self.target_loc).to_axis(),
        )
        .ok_or(RoutePlanError::OtherError("no viable powerslide turn"))?;

        let straight = GroundStraightPlanner::new(
            self.target_loc,
            asap,
            first_pass.segment.duration() - blueprint.duration,
            StraightMode::Real,
        )
        .plan(start_time, start, scenario)?;
        let straight_end = straight.segment.end();
        let slide = PowerslideTurn::new(blueprint, straight_end.boost);

        ChainedPlanner::new(
            Box::new(StaticPlanner::new(straight)),
            Some(Box::new(StaticPlanner::new(RoutePlan {
                segment: Box::new(slide),
                next: None,
            }))),
        )
        .plan(start_time, start, scenario)
    }
}
