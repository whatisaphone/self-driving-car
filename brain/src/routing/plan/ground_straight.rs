use common::{prelude::*, rl};
use nalgebra::Point2;
use ordered_float::NotNan;
use routing::{
    models::{CarState, RoutePlanError, RoutePlanner, RouteStep, SegmentPlan},
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{Chain, ForwardDodge, Straight, StraightMode},
};
use simulate::{Car1D, CarForwardDodge, CarForwardDodge1D};
use strategy::Scenario;

#[derive(Clone, new)]
pub struct StraightSimple {
    target_loc: Point2<f32>,
    target_time: f32,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
}

impl RoutePlanner for StraightSimple {
    fn plan(
        &self,
        _start_time: f32,
        start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RouteStep, RoutePlanError> {
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
            StraightMode::Fake,
        );
        Ok(RouteStep {
            segment: Box::new(segment),
            next: None,
        })
    }
}

/// Calculate a ground interception of the ball with a single dodge.
#[derive(Clone, new)]
pub struct StraightWithDodge {
    target_loc: Point2<f32>,
    target_time: f32,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
}

impl RoutePlanner for StraightWithDodge {
    fn plan(
        &self,
        _start_time: f32,
        start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);
        guard!(
            start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let dodges =
            calc_feasible_straight_dodges(start, self.target_loc, self.target_time, self.end_chop);
        let dodge = dodges
            .into_iter()
            .min_by_key(|d| NotNan::new(d.time_to_target).unwrap())
            .ok_or(RoutePlanError::MovingTooFast)?;

        let before = Straight::new(
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            start.loc.to_2d()
                + (self.target_loc - start.loc.to_2d()).normalize() * dodge.approach_distance,
            StraightMode::Real,
        );
        let dodge = ForwardDodge::new(before.end(), dodge.dodge);
        let after = Straight::new(
            dodge.end().loc.to_2d(),
            dodge.end().vel.to_2d(),
            dodge.end().boost,
            self.target_loc,
            StraightMode::Fake,
        );
        let segment = Chain::new(vec![Box::new(before), Box::new(dodge), Box::new(after)]);
        Ok(RouteStep {
            segment: Box::new(segment),
            next: None,
        })
    }
}

///
/// `end_chop` is the amount of time that should be left after the dodge for
/// recovery, shooting, etc.
fn calc_feasible_straight_dodges(
    start: &CarState,
    target_loc: Point2<f32>,
    target_time: f32,
    end_chop: f32,
) -> Vec<StraightDodge> {
    // Performance knob
    const GRANULARITY: f32 = 0.1;

    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut car = Car1D::new(start.vel.to_2d().norm()).with_boost(start.boost);
    let mut result = Vec::new();

    loop {
        car.multi_step(GRANULARITY, rl::PHYSICS_DT, 1.0, true);
        let dodge = CarForwardDodge::calc_1d(car.speed());
        let mut car2 = Car1D::new(dodge.end_speed).with_boost(car.boost());
        car2.multi_step(end_chop, rl::PHYSICS_DT, 1.0, true);

        let time = car.time() + dodge.duration() + car2.time();
        if time > target_time {
            break; // We would get there too slowly.
        }

        let target_traveled = (target_loc - start.loc.to_2d()).norm() - RADII;
        if car.distance_traveled() + dodge.end_dist + car2.distance_traveled() >= target_traveled {
            break; // The dodge would send us too far.
        }

        // To calculate the "best" dodge, they all need to be compared on equal terms.
        // The equal term I chose is the time needed to reach the target.
        while car.distance_traveled() + dodge.end_dist + car2.distance_traveled() < target_traveled
        {
            car2.step(rl::PHYSICS_DT, 1.0, true);
        }
        let time_to_target = car.time() + dodge.duration() + car2.time();

        result.push(StraightDodge {
            approach_distance: car.distance_traveled(),
            dodge,
            time_to_target,
        });
    }

    result
}

struct StraightDodge {
    approach_distance: f32,
    dodge: CarForwardDodge1D,
    time_to_target: f32,
}
