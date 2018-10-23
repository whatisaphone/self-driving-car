use behavior::Predicate;
use chip::max_curvature;
use common::ext::ExtendUnitVector3;
use maneuvers::GroundedHit;
use nalgebra::Point2;
use predict::{estimate_intercept_car_ball_2, intercept::NaiveIntercept};
use routing::{
    models::{CarState, RoutePlan, SegmentPlan},
    recover::{IsSkidding, NotOnFlatGround},
    segments::{SimpleArc, Straight},
};
use strategy::Context;
use utils::geometry::{circle_point_tangents, ExtendPoint3, ExtendVector2, ExtendVector3};

pub struct RoutePlanInfo {
    pub plan: RoutePlan,
    pub intercept: NaiveIntercept,
}

#[derive(Debug)]
pub enum RoutePlanError {
    MustBeOnFlatGround,
    MustNotBeSkidding,
    UnknownIntercept,
    OtherError(&'static str),
}

pub fn ground_intercept(ctx: &mut Context) -> Result<RoutePlanInfo, RoutePlanError> {
    if NotOnFlatGround.evaluate(ctx) {
        return Err(RoutePlanError::MustBeOnFlatGround);
    }
    if IsSkidding.evaluate(ctx) {
        return Err(RoutePlanError::MustNotBeSkidding);
    }

    let me = ctx.me();

    // Naive first pass to get a rough location.
    let guess =
        estimate_intercept_car_ball_2(ctx, me, |ball| ball.loc.z < GroundedHit::max_ball_z());
    let guess = some_or_else!(guess, {
        return Err(RoutePlanError::UnknownIntercept);
    });

    let plan = simple_drive_towards(&me.into(), guess.ball_loc.to_2d())?;
    Ok(RoutePlanInfo {
        plan,
        intercept: guess,
    })
}

fn simple_drive_towards(
    start: &CarState,
    target_loc: Point2<f32>,
) -> Result<RoutePlan, RoutePlanError> {
    let turn = turn_towards(start, target_loc)?;
    let turn_end = match turn {
        Some(ref segment) => segment.end(),
        None => start.clone(),
    };

    let straight = Box::new(Straight::new(
        turn_end.loc.to_2d(),
        turn_end.vel.to_2d(),
        target_loc,
    ));

    Ok(RoutePlan::new(
        vec![turn, Some(straight)]
            .into_iter()
            .filter_map(|x| x)
            .collect(),
    ))
}

fn turn_towards(
    start: &CarState,
    target_loc: Point2<f32>,
) -> Result<Option<Box<SegmentPlan>>, RoutePlanError> {
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

    let segment = SimpleArc::new(turn_center, turn_radius, start_loc, start_vel, tangent)
        .map_err(|err| RoutePlanError::OtherError(err.to_str()))?;
    Ok(Some(Box::new(segment)))
}
