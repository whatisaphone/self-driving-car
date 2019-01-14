use crate::{
    routing::{
        models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
        plan::ground_powerslide::GroundPowerslideTurn,
        recover::{IsSkidding, NotOnFlatGround},
        segments::JumpAndDodge,
    },
    strategy::BoostPickup,
};
use common::prelude::*;
use nalgebra::Point2;
use nameof::name_of_type;
use ordered_float::NotNan;
use std::f32::consts::PI;

#[derive(Clone)]
pub struct GetDollar {
    destination_hint: Point2<f32>,
    target_face: Point2<f32>,
}

impl GetDollar {
    pub fn new(destination_hint: Point2<f32>) -> Self {
        Self {
            destination_hint,
            target_face: destination_hint,
        }
    }

    pub fn target_face(mut self, target_face: Point2<f32>) -> Self {
        self.target_face = target_face;
        self
    }
}

impl RoutePlanner for GetDollar {
    fn name(&self) -> &'static str {
        name_of_type!(GetDollar)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
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

        if let Some(plan) = Self::quick_flip(ctx, dump, pickup) {
            return Ok(plan);
        }

        self.powerslide_turn(ctx, dump, pickup)
    }
}

impl GetDollar {
    /// If we're super close and not moving, just flip to it. Right now path
    /// routing is not good enough to do anything smarter.
    fn quick_flip(
        ctx: &PlanningContext<'_, '_>,
        _dump: &mut PlanningDump<'_>,
        pickup: &BoostPickup,
    ) -> Option<RoutePlan> {
        if ctx.start.vel.norm() >= 500.0 {
            return None;
        }
        let dist = (ctx.start.loc.to_2d() - pickup.loc).norm();
        if dist >= 750.0 {
            return None;
        }

        let direction = ctx
            .start
            .forward_axis_2d()
            .rotation_to(&(pickup.loc - ctx.start.loc.to_2d()).to_axis());

        Some(RoutePlan {
            segment: Box::new(JumpAndDodge::new(ctx.start.clone(), direction)),
            next: None,
        })
    }

    fn powerslide_turn(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
        pickup: &BoostPickup,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: pickup.loc,
        });

        if (pickup.loc - ctx.start.loc.to_2d()).norm() < 500.0
            && ctx.start.vel.to_2d().norm() < 100.0
            && ctx
                .start
                .forward_axis_2d()
                .angle_to(&(pickup.loc - ctx.start.loc.to_2d()).to_axis())
                .abs()
                >= PI / 3.0
        {
            return Err(RoutePlanError::OtherError("TODO: easier to flip to pad"));
        }

        // Minor hack – if we're retreating to grab boost, chances are we want to be
        // defensive. Force facing our goal because usually the other way ends up being
        // awkward.
        let mut target_face = self.target_face;
        if (ctx.game.own_goal().center_2d.y - pickup.loc.y).abs() < 1500.0
            && (pickup.loc.y - ctx.start.loc.y).abs() >= 2000.0
        {
            dump.log(self, "overriding target_face to be defensive");
            target_face = ctx.game.own_goal().center_2d;
        }

        GroundPowerslideTurn::new(pickup.loc, target_face, None).plan(ctx, dump)
    }

    fn chooose_pickup<'a>(
        ctx: &'a PlanningContext<'_, '_>,
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
            let along_penalty = line_span.norm() * 0.75 - along_dist;
            let ortho_penalty = ortho_dist * 2.0;
            let detour_penalty = match along_dist {
                d if d < 0.0 => -d,
                d if d >= line_span.norm() => d - line_span.norm(),
                _ => 0.0,
            };
            let penalty = along_penalty.powi(2) + ortho_penalty.powi(2) + detour_penalty.powi(2);
            NotNan::new(penalty).unwrap()
        })
    }
}
