use common::{prelude::*, rl};
use nalgebra::Point2;
use ordered_float::NotNan;
use routing::{
    models::{CarState, RoutePlan, RoutePlanError, RoutePlanner, SegmentPlan},
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{Chain, ForwardDodge, Straight, StraightMode},
};
use simulate::{Car1D, CarForwardDodge, CarForwardDodge1D};
use strategy::Scenario;

#[derive(Clone, new)]
pub struct GroundStraightPlanner {
    target_loc: Point2<f32>,
    target_time: f32,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for GroundStraightPlanner {
    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        let simple =
            StraightSimple::new(self.target_loc, self.target_time, self.end_chop, self.mode);
        let with_dodge =
            StraightWithDodge::new(self.target_loc, self.target_time, self.end_chop, self.mode);

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

fn fastest(steps: impl Iterator<Item = RoutePlan>) -> RoutePlan {
    steps
        .min_by_key(|s| NotNan::new(s.segment.duration()).unwrap())
        .unwrap()
}

#[derive(Clone, new)]
struct StraightSimple {
    target_loc: Point2<f32>,
    target_time: f32,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for StraightSimple {
    fn plan(
        &self,
        _start_time: f32,
        start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);
        guard!(
            start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let segment = Straight::new(
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            self.target_loc,
            self.mode,
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}

/// Calculate a ground interception of the ball with a single dodge.
#[derive(Clone, new)]
struct StraightWithDodge {
    target_loc: Point2<f32>,
    target_time: f32,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for StraightWithDodge {
    fn plan(
        &self,
        _start_time: f32,
        start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);
        guard!(
            start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let dodges = StraightDodgeCalculator::new(
            start.clone(),
            self.target_loc,
            self.target_time,
            self.end_chop,
        )
        .collect();
        let dodge = dodges
            .into_iter()
            .min_by_key(|d| NotNan::new(d.score).unwrap())
            .ok_or(RoutePlanError::MovingTooFast)?;

        let before = Straight::new(
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            start.loc.to_2d()
                + (self.target_loc - start.loc.to_2d()).normalize() * dodge.approach_distance,
            StraightMode::Asap,
        );
        let dodge = ForwardDodge::new(before.end(), dodge.dodge);
        let after = Straight::new(
            dodge.end().loc.to_2d(),
            dodge.end().vel.to_2d(),
            dodge.end().boost,
            self.target_loc,
            self.mode,
        );
        let segment = Chain::new(vec![Box::new(before), Box::new(dodge), Box::new(after)]);
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}

/// Calculate motions consisting of straight, then dodge, then straight again.
#[derive(new)]
struct StraightDodgeCalculator {
    start: CarState,
    target_loc: Point2<f32>,
    target_time: f32,
    end_chop: f32,
}

impl StraightDodgeCalculator {
    pub fn collect(&self) -> Vec<StraightDodge> {
        // Performance knob
        const GRANULARITY: f32 = 0.1;

        let mut car = Car1D::new(self.start.vel.to_2d().norm()).with_boost(self.start.boost);
        let mut result = Vec::new();

        while car.time() < self.target_time {
            car.multi_step(GRANULARITY, rl::PHYSICS_DT, 1.0, true);
            if let Some(dodge) = self.evaluate(&car) {
                result.push(dodge);
            }
        }

        result
    }

    fn evaluate(&self, approach: &Car1D) -> Option<StraightDodge> {
        let dodge = CarForwardDodge::calc_1d(approach.speed());

        // `end_chop` is "dead time" that the caller requested we leave available for
        // the subsequent maneuver. Coasting on landing is the most conservative case.
        let mut dodge_end = Car1D::new(dodge.end_speed).with_boost(approach.boost());
        dodge_end.multi_step(self.end_chop, rl::PHYSICS_DT, 0.0, false);
        let dodge_end = dodge_end;

        // Now we know where this dodge would take us. Let's check if it meets the
        // requirements:

        // Check if we can even complete the dodge by the target time.
        let total_time = approach.time() + dodge.duration() + dodge_end.time();
        if total_time > self.target_time {
            return None;
        }

        // Check that we don't land past the target.
        let target_traveled = (self.target_loc - self.start.loc.to_2d()).norm();
        let total_dist =
            approach.distance_traveled() + dodge.end_dist + dodge_end.distance_traveled();
        if total_dist >= target_traveled {
            return None;
        }

        // Now simulate coasting after the landing. If we pass the target before the
        // target time, dodging made us go too fast.
        let mut coast = Car1D::new(dodge.end_speed).with_boost(approach.boost());
        coast.multi_step(self.target_time - total_time, rl::PHYSICS_DT, 0.0, false);
        if total_dist + coast.distance_traveled() > target_traveled {
            return None;
        }

        // To calculate the "best" dodge, they all need to be compared on equal terms.
        // The equal term I choose is the minimum time needed to reach the target.
        let mut blitz = Car1D::new(dodge.end_speed).with_boost(approach.boost());
        while total_dist + blitz.distance_traveled() < target_traveled {
            blitz.step(rl::PHYSICS_DT, 1.0, true);
        }
        let score = total_time + blitz.time();

        Some(StraightDodge {
            approach_distance: approach.distance_traveled(),
            dodge,
            score,
        })
    }
}

struct StraightDodge {
    approach_distance: f32,
    dodge: CarForwardDodge1D,
    score: f32,
}
