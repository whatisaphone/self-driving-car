use common::prelude::*;
use maneuvers::GroundedHit;
use predict::naive_ground_intercept;
use routing::{
    models::{CarState, RoutePlan, RoutePlanError, RoutePlanner},
    plan::{ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner},
    recover::{IsSkidding, NotOnFlatGround},
    segments::StraightMode,
};
use strategy::Scenario;

#[derive(Clone, new)]
pub struct GroundIntercept;

impl RoutePlanner for GroundIntercept {
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);

        // Naive first pass to get a rough location.
        let guess = naive_ground_intercept(
            scenario.ball_prediction().iter_delayed(start_time),
            start.loc,
            start.vel,
            start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        guard!(
            start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: guess.car_loc.to_2d(),
            },
        );

        TurnPlanner::new(
            guess.ball_loc.to_2d(),
            Some(Box::new(GroundInterceptStraight::new())),
        )
        .plan(start_time, start, scenario)
    }
}

#[derive(Clone, new)]
struct GroundInterceptStraight;

impl RoutePlanner for GroundInterceptStraight {
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);

        let guess = naive_ground_intercept(
            scenario.ball_prediction().iter_delayed(start_time),
            start.loc,
            start.vel,
            start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        guard!(
            start,
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
        .plan(start_time, start, scenario)
    }
}
