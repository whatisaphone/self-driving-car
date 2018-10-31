use chip::max_curvature;
use common::prelude::*;
use maneuvers::GroundedHit;
use nalgebra::Point2;
use ordered_float::{NotNan, OrderedFloat};
use plan::ball::{BallFrame, BallTrajectory};
use predict::naive_ground_intercept;
use routing::{
    models::{CarState, RoutePlanError, RoutePlanner, RoutePlannerCloneBox, RouteStep},
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{Chain, ForwardDodge, SimpleArc, Straight, Turn},
};
use simulate::{Car1D, CarForwardDodge, CarForwardDodge1D};
use strategy::Scenario;
use utils::geometry::circle_point_tangents;

macro_rules! guard {
    ($start:expr, $predicate:expr, $return:expr $(,)*) => {
        if $predicate.evaluate($start) {
            return Err($return);
        }
    };
}

#[derive(Clone, new)]
pub struct GroundIntercept;

impl RoutePlanner for GroundIntercept {
    fn plan(&self, start: &CarState, scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        // Naive first pass to get a rough location.
        let guess = naive_ground_intercept(
            scenario.ball_prediction().iter(),
            start.loc,
            start.vel,
            start.boost,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        )
        .ok_or_else(|| RoutePlanError::UnknownIntercept)?;

        TurnPlanner::new(
            guess.ball_loc.to_2d(),
            Box::new(GroundInterceptStraight::new()),
        )
        .plan(start, scenario)
    }
}

#[derive(Clone, new)]
pub struct GroundInterceptStraight;

impl RoutePlanner for GroundInterceptStraight {
    fn plan(&self, start: &CarState, scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        let guess = naive_ground_intercept(
            scenario.ball_prediction().iter(),
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
        let plans = planners.iter().map(|p| p.plan(start, scenario));
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
    fn plan(&self, start: &CarState, _scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
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
        );
        Ok(RouteStep {
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
}

impl RoutePlanner for StraightWithDodge {
    fn plan(&self, start: &CarState, scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);
        guard!(
            start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let dodges = calc_feasible_straight_dodges(
            start,
            scenario.ball_prediction(),
            self.end_chop,
            |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
        );
        let dodge = dodges
            .into_iter()
            .min_by_key(|d| OrderedFloat(d.time_to_target))
            .ok_or(RoutePlanError::MovingTooFast)?;

        let approach = Straight::new(
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            start.loc.to_2d() + start.vel.to_2d().normalize() * dodge.approach_distance,
        );
        let dodge = ForwardDodge::new(start.clone(), dodge.dodge);
        let segment = Chain::new(vec![Box::new(approach), Box::new(dodge)]);
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
    ball_prediction: &BallTrajectory,
    end_chop: f32,
    predicate: impl Fn(&BallFrame) -> bool,
) -> Vec<StraightDodge> {
    // Performance knob
    const GRANULARITY: f32 = 0.1;

    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut car = Car1D::new(start.vel.to_2d().norm()).with_boost(start.boost);
    let mut result = Vec::new();

    loop {
        car.multi_step(GRANULARITY, BallFrame::DT, 1.0, true);
        let dodge = CarForwardDodge::calc_1d(car.speed());
        let mut car2 = Car1D::new(dodge.end_speed).with_boost(car.boost());
        car2.multi_step(end_chop, BallFrame::DT, 1.0, true);

        let time = car.time() + dodge.duration() + car2.time();
        let ball = some_or_else!(ball_prediction.at_time(time), {
            break; // We've reached the end of the prediction.
        });
        if !predicate(ball) {
            continue;
        }
        let target_traveled = (ball.loc - start.loc).to_2d().norm() - RADII;
        if car.distance_traveled() + dodge.end_dist + car2.distance_traveled() >= target_traveled {
            break; // The dodge would send us too far.
        }

        // To calculate the "best" dodge, they all need to be compared on equal terms.
        // The equal term I chose is the time needed to reach the target.
        while car.distance_traveled() + dodge.end_dist + car2.distance_traveled() < target_traveled
        {
            car2.step(BallFrame::DT, 1.0, true);
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

#[derive(new)]
struct TurnPlanner {
    target_loc: Point2<f32>,
    next: Box<RoutePlanner>,
}

impl RoutePlannerCloneBox for TurnPlanner {
    fn clone_box(&self) -> Box<RoutePlanner> {
        Box::new(Self {
            target_loc: self.target_loc,
            next: self.next.clone_box(),
        })
    }
}

impl RoutePlanner for TurnPlanner {
    fn plan(&self, start: &CarState, scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        let turn = match calculate_circle_turn(start, self.target_loc)? {
            Some(x) => x,
            None => return self.next.plan(start, scenario),
        };
        let segment = Turn::new(
            start.clone(),
            self.target_loc,
            turn.center,
            turn.radius,
            turn.tangent,
        );
        Ok(RouteStep {
            segment: Box::new(segment),
            next: Some(self.next.clone_box()),
        })
    }
}

#[allow(dead_code)] // This basically works but I can't make use of it just yet.
#[derive(new)]
struct ArcTowards {
    target_loc: Point2<f32>,
    next: Box<RoutePlanner>,
}

impl RoutePlannerCloneBox for ArcTowards {
    fn clone_box(&self) -> Box<RoutePlanner> {
        Box::new(Self {
            target_loc: self.target_loc,
            next: self.next.clone_box(),
        })
    }
}

impl RoutePlanner for ArcTowards {
    fn plan(&self, start: &CarState, scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        let turn = match calculate_circle_turn(start, self.target_loc)? {
            Some(x) => x,
            None => return self.next.plan(start, scenario),
        };

        let segment = SimpleArc::new(
            turn.center,
            turn.radius,
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            turn.tangent,
        )
        .map_err(|err| RoutePlanError::OtherError(err.to_str()))?;
        Ok(RouteStep {
            segment: Box::new(segment),
            next: Some(self.next.clone_box()),
        })
    }
}

fn calculate_circle_turn(
    start: &CarState,
    target_loc: Point2<f32>,
) -> Result<Option<CircleTurn>, RoutePlanError> {
    let start_loc = start.loc.to_2d();
    let start_vel = start.vel.to_2d();
    let start_forward_axis = start.forward_axis().to_2d();
    let start_right_axis = start.right_axis().to_2d();

    // Check if we're already facing the target
    let turn_rot = start_forward_axis.rotation_to(&(target_loc - start_loc).to_axis());
    if turn_rot.angle().abs() < 2.0_f32.to_radians() {
        return Ok(None);
    }

    // Define a circle where our current location/rotation form a tangent.
    let speed = f32::max(500.0, start_vel.norm());
    let start_vel = start_forward_axis.as_ref() * speed;
    let turn_radius = 1.0 / max_curvature(speed);
    let turn_center =
        start_loc + start_right_axis.as_ref() * turn_rot.angle().signum() * turn_radius;

    // Figure out which tangent point is relevant for this route.
    let [tangent1, tangent2] = match circle_point_tangents(turn_center, turn_radius, target_loc) {
        Some(x) => x,
        None => {
            return Err(RoutePlanError::OtherError(
                "Turning radius not tight enough",
            ))
        }
    };
    let t1_rot = (start_loc - turn_center).rotation_to(tangent1 - turn_center);
    let t2_rot = (start_loc - turn_center).rotation_to(tangent2 - turn_center);
    let t1_vel = t1_rot * start_vel;
    let t2_vel = t2_rot * start_vel;
    let t1_dot = t1_vel.dot(&(target_loc - tangent1));
    let t2_dot = t2_vel.dot(&(target_loc - tangent2));
    let tangent = match (t1_dot >= 0.0, t2_dot >= 0.0) {
        (true, false) => tangent1,
        (false, true) => tangent2,
        _ => return Err(RoutePlanError::OtherError("!= 1 tangent?")),
    };
    Ok(Some(CircleTurn {
        center: turn_center,
        radius: turn_radius,
        tangent,
    }))
}

struct CircleTurn {
    center: Point2<f32>,
    radius: f32,
    tangent: Point2<f32>,
}
