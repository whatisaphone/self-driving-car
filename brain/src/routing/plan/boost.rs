use crate::{
    eeg::EEG,
    helpers::telepathy,
    routing::{
        models::{
            CarState2D, PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner,
        },
        plan::{
            ground_drive::GroundDrive, ground_jump_and_dodge::GroundJumpAndDodge,
            ground_powerslide::GroundPowerslideTurn, higher_order::StaticPlanner, ChainedPlanner,
        },
        recover::{IsSkidding, NotOnFlatGround},
        segments::Brake,
    },
    strategy::{BoostPickup, Context2, Goal},
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
    pickup_loc: Option<Point2<f32>>,
}

impl GetDollar {
    pub fn new(destination_hint: Point2<f32>) -> Self {
        Self {
            destination_hint,
            target_face: destination_hint,
            pickup_loc: None,
        }
    }

    pub fn pickup(mut self, pickup: &BoostPickup) -> Self {
        self.pickup_loc = Some(pickup.loc);
        self
    }

    pub fn target_face(mut self, target_face: Point2<f32>) -> Self {
        self.target_face = target_face;
        self
    }

    // If you could put scare quotes in function names, this is the first place they
    // would go.
    pub fn smart(ctx: &Context2<'_, '_>, eeg: &mut EEG) -> Self {
        let future_loc = ctx.scenario.ball_prediction().at_time_or_last(3.0).loc;
        let behind_ball = Vector2::new(0.0, ctx.game.own_goal().center_2d.y.signum() * 2500.0);
        let opponent_hit = telepathy::predict_enemy_hit_direction_2(ctx)
            .map(|dir| dir.into_inner() * 2500.0)
            .unwrap_or_else(Vector2::zeros);
        let hint = future_loc.to_2d() + behind_ball + opponent_hit;

        eeg.log_pretty(name_of_type!(GetDollar), "opponent_hit", opponent_hit);
        eeg.log_pretty(name_of_type!(GetDollar), "hint", hint);

        Self::new(hint).target_face(future_loc.to_2d())
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

        let pickup = match self.chooose_pickup(ctx, self.destination_hint) {
            Some(p) => p,
            None => return Err(RoutePlanError::OtherError("no pickup found")),
        };

        if let Some(plan) = Self::quick_flip(ctx, dump, &pickup) {
            return Ok(plan);
        }

        self.powerslide_turn(ctx, dump, &pickup)
    }
}

impl GetDollar {
    /// If we're super close and not moving, just flip to it. Right now path
    /// routing is not good enough to do anything smarter.
    fn quick_flip(
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
        pickup: &BoostPickup,
    ) -> Option<RoutePlan> {
        let dist = (ctx.start.loc.to_2d() - pickup.loc).norm();
        if dist >= 750.0 {
            return None;
        }

        let direction = ctx
            .start
            .forward_axis_2d()
            .rotation_to(&(pickup.loc - ctx.start.loc.to_2d()).to_axis());
        if direction.angle().abs() < PI / 2.0 && ctx.start.vel_2d().norm() >= 500.0 {
            return None;
        }

        let mut planners = Vec::<Box<dyn RoutePlanner>>::new();

        // Brake first. This prevents the case where we are trying to get a boost that
        // isn't up yet, and we pass it and try to get it again with a wide, awkward
        // turn. Ideally we would not do this, but sadly we do. Without this, we always
        // end up driving up the wall. The next best thing is to brake and flop in place
        // until it spawns, and this makes that happen.
        let state = CarState2D {
            loc: ctx.start.loc.to_2d(),
            rot: ctx.start.rot.to_2d(),
            vel: ctx.start.vel.to_2d(),
            boost: ctx.start.boost,
        };
        planners.push(Box::new(StaticPlanner::new(RoutePlan {
            segment: Box::new(Brake::new(state, 250.0)),
            next: None,
        })));

        planners.push(Box::new(GroundJumpAndDodge::new(pickup.loc)));

        ChainedPlanner::chain(planners).plan(ctx, dump).ok()
    }

    fn powerslide_turn(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
        pickup: &BoostPickup,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_face,
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
        &self,
        ctx: &'a PlanningContext<'_, '_>,
        destination_hint: Point2<f32>,
    ) -> Option<BoostPickup> {
        if let Some(pickup_loc) = self.pickup_loc {
            return Some(BoostPickup { loc: pickup_loc });
        }

        Self::choose_pickup(
            ctx.game.boost_dollars().iter(),
            &CarState2D {
                loc: ctx.start.loc_2d(),
                rot: ctx.start.rot_2d(),
                vel: ctx.start.vel_2d(),
                boost: ctx.start.boost,
            },
            destination_hint,
            ctx.game.enemy_goal(),
        )
        .cloned()
    }

    pub fn choose_pickup<'a>(
        pickups: impl IntoIterator<Item = &'a BoostPickup>,
        start: &CarState2D,
        destination_hint: Point2<f32>,
        enemy_goal: &Goal,
    ) -> Option<&'a BoostPickup> {
        // Draw a line segment, and try to find a pickup along that segment.
        let line_start_loc = start.loc;
        let line_end_loc = destination_hint;
        let line_span = line_end_loc - line_start_loc;

        pickups.into_iter().min_by_key(|pickup| {
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
            let convenience_penalty =
                along_penalty.powi(2) + ortho_penalty.powi(2) + detour_penalty.powi(2);

            let is_enemy_boost = (pickup.loc.y - enemy_goal.center_2d.y).abs() < 2000.0;
            let positioning_penalty = if is_enemy_boost {
                let danger_distance = (pickup.loc - start.loc).dot(&-enemy_goal.normal_2d);
                let threshold = 3000.0;
                // Cube the distance beyond the threshold. That is a VERY STIFF penalty,
                // essentially a hard rejection.
                (danger_distance - threshold).max(0.0).powi(3)
            } else {
                0.0
            };
            let flatfoot_enemy_corner_penalty = if is_enemy_boost {
                // If we're in the enemy corner, facing the enemy back wall… it's just bad all
                // around. Get out of there.
                let defense_angle = start.forward_axis().angle_to(&enemy_goal.normal_2d).abs();
                let facing_factor =
                    linear_interpolate(&[PI * 0.5, PI * 0.75], &[0.0, 2500.0], defense_angle);
                facing_factor.powi(3)
            } else {
                0.0
            };

            let penalty = convenience_penalty + positioning_penalty + flatfoot_enemy_corner_penalty;
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
        if pickup.loc.y < 1.0
            && (approach.angle_to(&-Vector2::y_axis()).abs() < threshold
                || approach.angle_to(&Vector2::y_axis()).abs() < threshold)
        {
            return Point2::new(pickup.loc.x, -ctx.start.loc.y.signum() * 1000.0);
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
