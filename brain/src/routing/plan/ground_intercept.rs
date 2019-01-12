use crate::{
    behavior::strike::GroundedHit,
    plan::ball::{BallFrame, BallTrajectory},
    predict::naive_ground_intercept_2,
    routing::{
        models::{
            CarState, PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner,
        },
        plan::{
            ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner,
            higher_order::ChainedPlanner,
        },
        recover::{IsSkidding, NotOnFlatGround},
        segments::StraightMode,
    },
};
use common::prelude::*;
use derive_new::new;
use nameof::name_of_type;

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
        name_of_type!(GroundIntercept)
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

        // Naive first pass to get a rough location.
        let guess = Self::calc_intercept(&ctx.start, ctx.ball_prediction)
            .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        dump.log_pretty(self, "guess ball loc", guess.loc.to_2d());

        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: guess.loc.to_2d(),
        });

        let turn = TurnPlanner::new(guess.loc.to_2d(), None).plan(ctx, dump)?;

        let end_chop = 0.5;
        let straight = GroundStraightPlanner::new(
            guess.loc.to_2d(),
            Some(guess.t - turn.segment.duration()),
            end_chop,
            StraightMode::Fake,
        )
        .allow_dodging(self.allow_dodging);

        Ok(ChainedPlanner::join_planner(turn, Some(Box::new(straight))))
    }
}

impl GroundIntercept {
    pub fn calc_intercept<'ball>(
        start: &CarState,
        ball_prediction: &'ball BallTrajectory,
    ) -> Option<&'ball BallFrame> {
        naive_ground_intercept_2(start, ball_prediction, |ball| {
            ball.loc.z < GroundedHit::MAX_BALL_Z
        })
        .map(|i| ball_prediction.at_time(i.time).unwrap())
    }
}
