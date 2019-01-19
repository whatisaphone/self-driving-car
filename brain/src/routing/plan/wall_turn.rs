use crate::{
    routing::{
        models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
        plan::{ground_turn::calculate_circle_turn, wall_utils::which_surface},
        segments::{NullSegment, WallTurn},
    },
    utils::geometry::flattener::Flattener,
};
use nalgebra::Point3;
use nameof::name_of_type;
use std::f32::consts::PI;

const SLOWEST_TURNING_SPEED: f32 = 900.0;

#[derive(Clone)]
pub struct WallTurnPlanner {
    target_loc: Point3<f32>,
    maximum_turn_angle: f32,
}

impl WallTurnPlanner {
    pub fn new(target_loc: Point3<f32>) -> Self {
        Self {
            target_loc,
            maximum_turn_angle: 2.0 * PI,
        }
    }

    pub fn maximum_turn_angle(mut self, maximum_turn_angle: f32) -> Self {
        self.maximum_turn_angle = maximum_turn_angle;
        self
    }
}

impl RoutePlanner for WallTurnPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(WallTurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        let start_surface = which_surface(ctx.game.pitch(), &ctx.start.loc)
            .map_err(|_| RoutePlanError::CannotOperateWall)?;
        let target_surface = which_surface(ctx.game.pitch(), &self.target_loc)
            .map_err(|_| RoutePlanError::CannotOperateWall)?;
        let target_to_start = target_surface
            .unfold(start_surface)
            .map_err(|_| RoutePlanError::CannotOperateWall)?;
        let start_to_ground = start_surface
            .unfold(&ctx.game.pitch().ground())
            .map_err(|_| RoutePlanError::CannotOperateWall)?;
        let start_to_2d = Flattener::new(start_to_ground);
        let target_to_2d = Flattener::new(start_to_ground * target_to_start);

        let ground_start = start_to_ground * ctx.start.loc;
        if ground_start.z < 0.0 {
            // If our unfolded position is under the ground, most likely we're inside a goal
            // on the wall. There's no way anything sane can happen from here, so abort.
            return Err(RoutePlanError::CannotOperateWall);
        }

        let start = ctx.start.flatten(&start_to_2d);
        let target_loc = target_to_2d * self.target_loc;

        let turn_radius = 1.0 / chip::max_curvature(start.vel.norm().max(SLOWEST_TURNING_SPEED));
        let turn = match calculate_circle_turn(&start, turn_radius, target_loc)? {
            Some(x) => x,
            None => {
                return Ok(RoutePlan {
                    segment: Box::new(NullSegment::new(ctx.start.clone())),
                    next: None,
                });
            }
        };

        if turn.sweep().abs() >= self.maximum_turn_angle {
            return Err(RoutePlanError::TurnAngleTooLarge);
        }

        let segment = WallTurn::new(
            ctx.start.clone(),
            *start_surface,
            start_to_2d,
            turn.center,
            turn.radius,
            turn.tangent,
            target_loc,
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}
