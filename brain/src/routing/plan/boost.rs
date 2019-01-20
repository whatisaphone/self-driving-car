use crate::{
    routing::{
        models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
        plan::{ground_drive::GroundDrive, ground_powerslide::GroundPowerslideTurn},
        recover::{IsSkidding, NotOnFlatGround},
        segments::JumpAndDodge,
    },
    strategy::BoostPickup,
};
use common::prelude::*;
use nalgebra::{Point2, Vector2};
use nameof::name_of_type;
use ordered_float::NotNan;
use simulate::{linear_interpolate, Car1D};
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

        let travel_axis = (pickup.loc - ctx.start.loc_2d()).to_axis();
        let target_face = self.powerslide_target_face(ctx, pickup);
        let turn_angle = travel_axis.angle_to(&(target_face - pickup.loc).to_axis());
        if turn_angle.abs() < PI / 3.0 {
            let planner = GroundDrive::new(pickup.loc);
            planner.plan(ctx, dump)
        } else {
            let planner = GroundPowerslideTurn::new(pickup.loc, target_face, None);
            planner.plan(ctx, dump)
        }
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

    fn powerslide_target_face(
        &self,
        ctx: &PlanningContext<'_, '_>,
        pickup: &BoostPickup,
    ) -> Point2<f32> {
        let approach = pickup.loc - ctx.start.loc.to_2d();

        // The faster we're moving, the more likely we are to force the slide direction
        // away from the wall. This way we hopefully avoid sliding up the wall like an
        // idiot.
        let threshold = linear_interpolate(
            &[500.0, 4000.0],
            &[PI / 12.0, PI / 4.0],
            wild_guess_approach_speed(ctx, pickup),
        );

        // Midfield boost pads
        if pickup.loc.y < 1.0 {
            if approach.angle_to(&-Vector2::y_axis()).abs() < threshold
                || approach.angle_to(&Vector2::y_axis()).abs() < threshold
            {
                return Point2::new(pickup.loc.x, -ctx.start.loc.y.signum() * 1000.0);
            }
        }

        // Corner boost pads
        if pickup.loc.y >= 1.0 {
            if approach.angle_to(&-Vector2::y_axis()).abs() < threshold
                || approach.angle_to(&Vector2::y_axis()).abs() < threshold
            {
                return Point2::new(0.0, pickup.loc.y);
            } else if approach.angle_to(&-Vector2::x_axis()).abs() < threshold
                || approach.angle_to(&Vector2::x_axis()).abs() < threshold
            {
                return Point2::new(pickup.loc.x, 0.0);
            }
        }

        self.target_face
    }
}

fn wild_guess_approach_speed(ctx: &PlanningContext<'_, '_>, pickup: &BoostPickup) -> f32 {
    let slide_chop = 500.0;
    let travel = pickup.loc - ctx.start.loc_2d();
    let approach_dist = travel.norm() - slide_chop;
    if approach_dist <= 0.0 {
        return ctx.start.vel_2d().norm();
    }

    let mut car = Car1D::new()
        .with_speed(ctx.start.vel.norm())
        .with_boost(ctx.start.boost);
    car.advance_by_distance(approach_dist, 1.0, true);
    car.speed()
}
