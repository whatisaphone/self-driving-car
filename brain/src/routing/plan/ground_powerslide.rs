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

/// This is the part after the car is facing the right way. It consists of a
/// straight segment, then a slide segment.
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

        let asap = 0.0; // TODO: teach GroundStraightPlanner how to not care about the exact time of
                        // arrival.
        let throttle = 1.0;

        // First pass to estimate the approach
        let (first_pass_blueprint, rot_by) = {
            let end_chop = 1.0; // Leave some time for the actual slide.
            let first_pass =
                GroundStraightPlanner::new(self.target_loc, asap, end_chop, StraightMode::Asap)
                    .plan(start_time, start, scenario)?;

            let slide_start = first_pass.segment.end();
            let end_rot = CAR_LOCAL_FORWARD_AXIS_2D
                .rotation_to(&(self.target_face - self.target_loc).to_axis());
            let rot_by = start.rot.to_2d().rotation_to(&end_rot).angle();
            let blueprint = CarPowerslideTurn::evaluate(
                slide_start.loc.to_2d(),
                slide_start.vel.to_2d(),
                throttle,
                rot_by,
            )
            .ok_or(RoutePlanError::OtherError("no viable powerslide turn"))?;
            (blueprint, rot_by)
        };

        let slide_displacement = first_pass_blueprint.end_loc - first_pass_blueprint.start_loc;
        let fudge = 0.75; // This planner is inaccurate at short range. If we fudge the location, we're
                          // likely to slide through the target rather than missing it entirely, which is
                          // an improvement :)
        let slide_start_loc = self.target_loc - slide_displacement * fudge;
        let straight = GroundStraightPlanner::new(slide_start_loc, asap, 0.0, StraightMode::Asap)
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
