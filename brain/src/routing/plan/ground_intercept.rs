use common::prelude::*;
use maneuvers::GroundedHit;
use predict::naive_ground_intercept;
use routing::{
    models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
    plan::{
        ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner,
        higher_order::ChainedPlanner, pathing,
    },
    recover::{IsSkidding, NotOnFlatGround},
    segments::StraightMode,
};

#[derive(Clone, new)]
pub struct GroundIntercept;

impl RoutePlanner for GroundIntercept {
    fn name(&self) -> &'static str {
        stringify!(GroundIntercept)
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

        // Naive first pass to get a rough location.
        let guess = naive_ground_intercept(
            ctx.ball_prediction.iter(),
            ctx.start.loc,
            ctx.start.vel,
            ctx.start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        dump.log_pretty(self, "guess ball loc", guess.ball_loc.to_2d());

        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: guess.car_loc.to_2d(),
            },
        );

        let turn = TurnPlanner::new(guess.ball_loc.to_2d(), None).plan(ctx, dump)?;
        dump.log_plan(self, &turn);
        let turn = match pathing::avoid_smacking_goal_wall(&turn.segment.end()) {
            None => turn,
            Some(divert) => {
                dump.log(self, "diverting due to avoid_smacking_goal_wall");
                ChainedPlanner::new(divert, Some(Box::new(self.clone()))).plan(ctx, dump)?
            }
        };
        Ok(ChainedPlanner::join_planner(
            turn,
            Some(Box::new(GroundInterceptStraight::new())),
        ))
    }
}

#[derive(Clone, new)]
struct GroundInterceptStraight;

impl RoutePlanner for GroundInterceptStraight {
    fn name(&self) -> &'static str {
        stringify!(GroundInterceptStraight)
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

        let guess = naive_ground_intercept(
            ctx.ball_prediction.iter(),
            ctx.start.loc,
            ctx.start.vel,
            ctx.start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        dump.log_pretty(self, "guess.ball_loc", guess.ball_loc.to_2d());

        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: guess.car_loc.to_2d(),
            },
        );

        let end_chop = 0.5;
        GroundStraightPlanner::new(
            guess.car_loc.to_2d(),
            guess.time,
            end_chop,
            StraightMode::Fake,
        )
        .plan(ctx, dump)
    }
}
