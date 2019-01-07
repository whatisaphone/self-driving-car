use crate::{
    routing::{
        models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
        plan::wall_utils::which_surface,
        segments::WallStraight,
    },
    utils::geometry::flattener::Flattener,
};
use derive_new::new;
use nalgebra::Point3;
use nameof::name_of_type;

#[derive(Clone, new)]
pub struct WallStraightPlanner {
    target_loc: Point3<f32>,
}

impl RoutePlanner for WallStraightPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(WallStraightPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
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

        let segment = WallStraight::new(
            ctx.start.clone(),
            self.target_loc,
            start_to_2d,
            target_to_2d,
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}
