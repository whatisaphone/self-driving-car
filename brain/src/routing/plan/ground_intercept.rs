use crate::{
    maneuvers::GroundedHit,
    predict::naive_ground_intercept,
    routing::{
        models::{PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner},
        plan::{
            ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner,
            higher_order::ChainedPlanner,
        },
        recover::{IsSkidding, NotOnFlatGround},
        segments::StraightMode,
    },
};
use common::prelude::*;

#[derive(Clone, new)]
pub struct GroundIntercept {
    #[new(value = "false")]
    allow_dodging: bool,
}

impl GroundIntercept {
    pub fn allow_dodging(mut self, allow_dodging: bool) -> Self {
        self.allow_dodging = allow_dodging;
        self
    }
}

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

        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: guess.car_loc.to_2d(),
        });

        let turn = TurnPlanner::new(guess.ball_loc.to_2d(), None).plan(ctx, dump)?;
        let straight = GroundInterceptStraight {
            allow_dodging: self.allow_dodging,
        };
        Ok(ChainedPlanner::join_planner(turn, Some(Box::new(straight))))
    }
}

#[derive(Clone)]
struct GroundInterceptStraight {
    allow_dodging: bool,
}

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

        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: guess.car_loc.to_2d(),
        });

        let end_chop = 0.5;
        GroundStraightPlanner::new(
            guess.car_loc.to_2d(),
            Some(guess.time),
            end_chop,
            StraightMode::Fake,
        )
        .allow_dodging(self.allow_dodging)
        .plan(ctx, dump)
    }
}
