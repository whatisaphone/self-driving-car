use common::prelude::*;
use nalgebra::Point2;
use ordered_float::NotNan;
use routing::{
    models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
    plan::ground_powerslide::GroundPowerslideTurn,
    recover::{IsSkidding, NotOnFlatGround},
};
use std::f32::consts::PI;
use strategy::BoostPickup;

#[derive(Clone)]
pub struct GetDollar {
    destination_hint: Point2<f32>,
}

impl GetDollar {
    pub fn new(destination_hint: Point2<f32>) -> Self {
        Self { destination_hint }
    }
}

impl RoutePlanner for GetDollar {
    fn name(&self) -> &'static str {
        stringify!(GetDollar)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );

        let pickup = match Self::chooose_pickup(ctx, self.destination_hint) {
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

        GroundPowerslideTurn::new(pickup.loc, self.destination_hint, None).plan(ctx, dump)
    }
}

impl GetDollar {
    fn chooose_pickup<'a>(
        ctx: &'a PlanningContext,
        destination_hint: Point2<f32>,
    ) -> Option<&'a BoostPickup> {
        Self::choose_pickup(
            ctx.game.boost_dollars().iter(),
            ctx.start.loc.to_2d(),
            destination_hint,
        )
    }

    pub fn choose_pickup<'a>(
        pickups: impl Iterator<Item = &'a BoostPickup>,
        start_loc: Point2<f32>,
        destination_hint: Point2<f32>,
    ) -> Option<&'a BoostPickup> {
        // Draw a line segment, and try to find a pickup along that segment.
        let line_start_loc = start_loc;
        let line_end_loc = destination_hint;
        let line_span = line_end_loc - line_start_loc;

        pickups.min_by_key(|pickup| {
            let along_dist = (pickup.loc - line_start_loc).dot(&line_span.to_axis());
            let ortho_dist = (pickup.loc - line_start_loc).dot(&line_span.ortho().to_axis());
            // Assume an "ideal" position 75% of the way down the line.
            let along_score = line_span.norm() * 0.75 - along_dist;
            BoostScore {
                invalid: along_dist < -250.0 || along_dist >= line_span.norm() + 250.0,
                score: NotNan::new(along_score.powi(2) + (ortho_dist * 2.0).powi(2)).unwrap(),
            }
        })
    }
}

/// Lower is better.
#[derive(Ord, PartialOrd, Eq, PartialEq)]
struct BoostScore {
    invalid: bool,
    score: NotNan<f32>,
}
