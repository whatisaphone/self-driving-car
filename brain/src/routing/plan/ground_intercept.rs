use crate::{
    behavior::strike::GroundedHit,
    helpers::{
        ball::{BallFrame, BallTrajectory},
        intercept::{naive_ground_intercept_2, naive_intercept_penalty},
    },
    routing::{
        models::{
            CarState, PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner,
        },
        plan::{
            ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner,
            higher_order::ChainedPlanner,
        },
        recover::{is_ball_directly_behind_car, IsSkidding, NotOnFlatGround},
        segments::StraightMode,
    },
};
use common::{prelude::*, Time};
use derive_new::new;
use nameof::name_of_type;

#[derive(Clone, new)]
pub struct GroundIntercept {
    #[new(value = "GroundInterceptAllowDodging::OnlyIfSlow")]
    allow_dodging: GroundInterceptAllowDodging,
}

#[derive(Copy, Clone)]
pub enum GroundInterceptAllowDodging {
    Yes,
    No,
    OnlyIfSlow,
}

impl GroundIntercept {
    pub fn allow_dodging(mut self, allow_dodging: bool) -> Self {
        self.allow_dodging = if allow_dodging {
            GroundInterceptAllowDodging::Yes
        } else {
            GroundInterceptAllowDodging::No
        };
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

        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: guess.loc.to_2d(),
        });

        dump.log_pretty(self, "guess ball time", Time(guess.t));
        dump.log_pretty(self, "guess ball loc", guess.loc.to_2d());

        if is_ball_directly_behind_car(ctx.ball_prediction, &ctx.start) {
            dump.log(self, "the ball is directly behind us; I can't even");
            return Err(RoutePlanError::TurningRadiusTooTight);
        }

        let turn = TurnPlanner::new(guess.loc.to_2d(), None).plan(ctx, dump)?;

        let mut straight_time = guess.t - turn.segment.duration();
        if straight_time < 0.0 {
            dump.log(
                self,
                "the turn takes too long, but let's pretend we didn't notice",
            );
            straight_time = 0.0;
        }
        let allow_dodging = match self.allow_dodging {
            GroundInterceptAllowDodging::Yes => true,
            GroundInterceptAllowDodging::No => false,
            GroundInterceptAllowDodging::OnlyIfSlow => {
                straight_time >= 3.0 && ctx.start.boost < 50.0
            }
        };
        let straight = GroundStraightPlanner::new(guess.loc.to_2d(), StraightMode::Fake)
            .target_time(straight_time)
            .end_chop(0.5)
            .allow_dodging(allow_dodging);

        Ok(ChainedPlanner::join_planner(turn, Some(Box::new(straight))))
    }
}

impl GroundIntercept {
    pub fn calc_intercept<'ball>(
        start: &CarState,
        ball_prediction: &'ball BallTrajectory,
    ) -> Option<&'ball BallFrame> {
        let intercept = naive_ground_intercept_2(start, ball_prediction, |ball| {
            ball.loc.z < GroundedHit::MAX_BALL_Z
        })?;
        let intercept = ball_prediction.at_time(intercept.time).unwrap();
        let penalty = naive_intercept_penalty(start, intercept);
        Some(ball_prediction.at_time_or_last(intercept.t + penalty))
    }
}
