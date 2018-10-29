use chip::max_curvature;
use common::ext::{ExtendPoint3, ExtendUnitVector3, ExtendVector2, ExtendVector3};
use maneuvers::GroundedHit;
use nalgebra::Point2;
use predict::naive_ground_intercept;
use routing::{
    models::{CarState, RoutePlanError, RoutePlanner, RoutePlannerCloneBox, RouteStep},
    recover::{IsSkidding, NotOnFlatGround},
    segments::{NullSegment, SimpleArc, Straight, Turn},
};
use strategy::Scenario;
use utils::geometry::circle_point_tangents;

macro_rules! guard {
    ($start:expr, $predicate:expr, $return:expr) => {
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

        let segment = Straight::new(
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            guess.ball_loc.to_2d(),
        );
        Ok(RouteStep {
            segment: Box::new(segment),
            next: None,
        })
    }
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
    fn plan(&self, start: &CarState, _scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        let (turn_center, turn_radius, tangent) =
            match calculate_circle_turn(start, self.target_loc)? {
                Some(x) => x,
                None => {
                    return Ok(RouteStep {
                        segment: Box::new(NullSegment::new(start.clone())),
                        next: Some(self.next.clone_box()),
                    })
                }
            };
        let segment = Turn::new(
            start.clone(),
            self.target_loc,
            turn_center,
            turn_radius,
            tangent,
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
    fn plan(&self, start: &CarState, _scenario: &Scenario) -> Result<RouteStep, RoutePlanError> {
        guard!(start, NotOnFlatGround, RoutePlanError::MustBeOnFlatGround);
        guard!(start, IsSkidding, RoutePlanError::MustNotBeSkidding);

        let (turn_center, turn_radius, tangent) =
            match calculate_circle_turn(start, self.target_loc)? {
                Some(x) => x,
                None => {
                    return Ok(RouteStep {
                        segment: Box::new(NullSegment::new(start.clone())),
                        next: Some(self.next.clone_box()),
                    })
                }
            };

        let segment = SimpleArc::new(
            turn_center,
            turn_radius,
            start.loc.to_2d(),
            start.vel.to_2d(),
            start.boost,
            tangent,
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
) -> Result<Option<(Point2<f32>, f32, Point2<f32>)>, RoutePlanError> {
    let start_loc = start.loc.to_2d();
    let start_vel = start.vel.to_2d();
    let start_forward_axis = start.forward_axis().to_2d();
    let start_right_axis = start.right_axis().to_2d();

    // Check if we're already facing the target
    let turn_rot = start_forward_axis.rotation_to(target_loc - start_loc);
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
    Ok(Some((turn_center, turn_radius, tangent)))
}
