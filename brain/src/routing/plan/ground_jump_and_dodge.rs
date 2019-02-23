use crate::routing::{
    models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
    recover::NotOnFlatGround,
    segments::JumpAndDodge,
};
use common::prelude::*;
use derive_new::new;
use nalgebra::Point2;
use nameof::name_of_type;

#[derive(Clone, new)]
pub struct GroundJumpAndDodge {
    target_loc: Point2<f32>,
}

impl RoutePlanner for GroundJumpAndDodge {
    fn name(&self) -> &'static str {
        name_of_type!(GroundJumpAndDodge)
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

        let direction = ctx
            .start
            .forward_axis_2d()
            .rotation_to(&(self.target_loc - ctx.start.loc.to_2d()).to_axis());

        dump.log_pretty(self, "direction", direction);

        Ok(RoutePlan {
            segment: Box::new(JumpAndDodge::new(ctx.start.clone(), direction)),
            next: None,
        })
    }
}
