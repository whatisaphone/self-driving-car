use common::{physics::CAR_LOCAL_FORWARD_AXIS_2D, prelude::*};
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
    fn name(&self) -> &'static str {
        stringify!(GroundPowerslideTurn)
    }

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

#[derive(Clone, new)]
pub struct GroundSimplePowerslideTurn {
    target_face: Point2<f32>,
}

impl RoutePlanner for GroundSimplePowerslideTurn {
    fn name(&self) -> &'static str {
        stringify!(GroundSimplePowerslideTurn)
    }

    fn plan(
        &self,
        _start_time: f32,
        start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(
            start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: self.target_face,
            },
        );

        let throttle = 1.0;
        let munged_start_loc = start.loc.to_2d() + start.vel.to_2d() * 0.5;
        let end_rot =
            CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&(self.target_face - munged_start_loc).to_axis());
        let rot_by = start.rot.to_2d().rotation_to(&end_rot).angle();
        let blueprint =
            CarPowerslideTurn::evaluate(start.loc.to_2d(), start.vel.to_2d(), throttle, rot_by)
                .ok_or(RoutePlanError::OtherError("no viable powerslide turn"))?;
        let slide = PowerslideTurn::new(blueprint, start.boost);

        Ok(RoutePlan {
            segment: Box::new(slide),
            next: None,
        })
    }
}

/// This is the part after the car is facing the right way. It consists of a
/// straight segment, then a slide segment.
#[derive(Clone, new)]
struct GroundPowerslideEssence {
    target_loc: Point2<f32>,
    target_face: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for GroundPowerslideEssence {
    fn name(&self) -> &'static str {
        stringify!(GroundPowerslideEssence)
    }

    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(
            start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: self.target_loc,
            },
        );
        guard!(
            start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let asap = 0.0; // TODO: teach GroundStraightPlanner how to not care about the exact time of
                        // arrival.
        let throttle = 1.0;

        let end_rot =
            CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&(self.target_face - self.target_loc).to_axis());
        let rot_by = start.rot.to_2d().rotation_to(&end_rot).angle();

        // First pass to estimate the approach angle
        let blueprint = {
            let end_chop = 1.0; // Leave some time for the actual slide.
            let straight =
                GroundStraightPlanner::new(self.target_loc, asap, end_chop, StraightMode::Asap)
                    .plan(start_time, start, scenario)?;
            let straight_end = straight.segment.end();

            CarPowerslideTurn::evaluate(
                straight_end.loc.to_2d(),
                straight_end.vel.to_2d(),
                throttle,
                rot_by,
            )
            .ok_or(RoutePlanError::OtherError("no viable powerslide turn"))?
        };

        // Second pass to estimate the approach velocity
        let blueprint = {
            let start_to_target = self.target_loc - start.loc.to_2d();
            let blueprint_span = blueprint.end_loc - blueprint.start_loc;
            let dist_reduction = blueprint_span.dot(&start_to_target.to_axis());
            let straight_dist = start_to_target.norm() - dist_reduction;
            let straight_end_loc = start.loc.to_2d() + start_to_target.normalize() * straight_dist;
            let straight =
                GroundStraightPlanner::new(straight_end_loc, asap, 0.0, StraightMode::Asap)
                    .plan(start_time, start, scenario)?;
            let straight_end = straight.segment.end();

            CarPowerslideTurn::evaluate(
                straight_end.loc.to_2d(),
                straight_end.vel.to_2d(),
                throttle,
                rot_by,
            )
            .ok_or(RoutePlanError::OtherError("no viable powerslide turn"))?
        };

        let straight_end_loc = self.target_loc - (blueprint.end_loc - blueprint.start_loc);
        let straight = GroundStraightPlanner::new(straight_end_loc, asap, 0.0, StraightMode::Asap)
            .plan(start_time, start, scenario)?;
        let straight_end = straight.segment.end();

        let blueprint = CarPowerslideTurn::evaluate(
            straight_end.loc.to_2d(),
            straight_end.vel.to_2d(),
            throttle,
            rot_by,
        )
        .ok_or(RoutePlanError::OtherError("no viable powerslide turn"))?;
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
