use common::prelude::*;
use maneuvers::GroundedHit;
use ordered_float::NotNan;
use predict::naive_ground_intercept;
use routing::{
    models::{CarState, RoutePlanError, RoutePlanner, RouteStep},
    plan::{
        ground_straight::{StraightSimple, StraightWithDodge},
        ground_turn::TurnPlanner,
    },
    recover::{IsSkidding, NotOnFlatGround},
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
    ) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        // Naive first pass to get a rough location.
        let guess = naive_ground_intercept(
            scenario.ball_prediction().iter_delayed(start_time),
            start.loc,
            start.vel,
            start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        TurnPlanner::new(
            guess.ball_loc.to_2d(),
            Some(Box::new(GroundInterceptStraight::new())),
        )
        .plan(start_time, start, scenario)
    }
}

#[derive(Clone, new)]
pub struct GroundInterceptStraight;

impl RoutePlanner for GroundInterceptStraight {
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        let guess = naive_ground_intercept(
            scenario.ball_prediction().iter_delayed(start_time),
            start.loc,
            start.vel,
            start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        let end_chop = 0.5;
        let simple = StraightSimple::new(guess.car_loc.to_2d(), guess.time, end_chop);
        let with_dodge = StraightWithDodge::new(guess.car_loc.to_2d(), guess.time, end_chop);

        let planners = [&simple as &RoutePlanner, &with_dodge];
        let plans = planners.iter().map(|p| p.plan(start_time, start, scenario));
        let plans = at_least_one_ok(plans)?;
        Ok(fastest(plans.into_iter()))
    }
}

fn at_least_one_ok<T, E>(results: impl Iterator<Item = Result<T, E>>) -> Result<Vec<T>, E> {
    let mut oks: Vec<T> = Vec::new();
    let mut error = None;
    for result in results {
        match result {
            Ok(x) => oks.push(x),
            Err(e) => error = Some(e),
        }
    }
    if oks.is_empty() {
        Err(error.unwrap())
    } else {
        Ok(oks)
    }
}

fn fastest(steps: impl Iterator<Item = RouteStep>) -> RouteStep {
    steps
        .min_by_key(|s| NotNan::new(s.segment.duration()).unwrap())
        .unwrap()
}
