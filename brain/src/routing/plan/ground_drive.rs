use crate::routing::{
    models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
    plan::{
        ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner,
        higher_order::ChainedPlanner,
    },
    recover::{IsSkidding, NotOnFlatGround},
    segments::StraightMode,
};
use nalgebra::Point2;

#[derive(Clone, new)]
pub struct GroundDrive {
    target_loc: Point2<f32>,
    #[new(value = "0.0")]
    end_chop: f32,
}

impl GroundDrive {
    pub fn end_chop(mut self, end_chop: f32) -> Self {
        self.end_chop = end_chop;
        self
    }
}

impl RoutePlanner for GroundDrive {
    fn name(&self) -> &'static str {
        stringify!(GroundDrive)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_loc", self.target_loc);

        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: self.target_loc,
            },
        );

        let turn = TurnPlanner::new(self.target_loc, None).plan(ctx, dump)?;
        let straight =
            GroundStraightPlanner::new(self.target_loc, None, self.end_chop, StraightMode::Asap);
        Ok(ChainedPlanner::join_planner(turn, Some(Box::new(straight))))
    }
}
