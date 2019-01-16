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
use nameof::name_of_type;

#[derive(Clone)]
pub struct GroundDrive {
    target_loc: Point2<f32>,
    end_chop: f32,
    straight_mode: StraightMode,
}

impl GroundDrive {
    pub fn new(target_loc: Point2<f32>) -> Self {
        Self {
            target_loc,
            end_chop: 0.0,
            straight_mode: StraightMode::Asap,
        }
    }

    pub fn end_chop(mut self, end_chop: f32) -> Self {
        self.end_chop = end_chop;
        self
    }

    pub fn straight_mode(mut self, straight_mode: StraightMode) -> Self {
        self.straight_mode = straight_mode;
        self
    }
}

impl RoutePlanner for GroundDrive {
    fn name(&self) -> &'static str {
        name_of_type!(GroundDrive)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_loc", self.target_loc);

        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });

        let turn = TurnPlanner::new(self.target_loc, None).plan(ctx, dump)?;
        let straight =
            GroundStraightPlanner::new(self.target_loc, self.straight_mode).end_chop(self.end_chop);
        Ok(ChainedPlanner::join_planner(turn, Some(Box::new(straight))))
    }
}
