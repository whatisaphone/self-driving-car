use behavior::Predicate;
use chip::max_curvature;
use common::{
    ext::ExtendUnitVector3,
    physics::{car_forward_axis_2d, CAR_LOCAL_FORWARD_AXIS_2D},
};
use maneuvers::GroundedHit;
use nalgebra::Point2;
use predict::{estimate_intercept_car_ball_2, intercept::NaiveIntercept, naive_ground_intercept};
use routing::{
    models::{CarState, RoutePlan, SegmentPlan},
    recover::{IsSkidding, NotOnFlatGround},
    segments::{PowerslideTurn, SimpleArc, Straight},
};
use simulate::CarPowerslideTurn;
use std::iter;
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
    let guess = estimate_intercept_car_ball_2(ctx, me, |ball| {
        ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0
    });
    let guess = some_or_else!(guess, {
        return Err(RoutePlanError::UnknownIntercept);
    });

    let plan = simple_drive_towards(&me.into(), guess.ball_loc.to_2d())?;
    Ok(RoutePlanInfo {
        plan,
        intercept: guess,
    })
}

#[allow(dead_code)] // Doesn't yet work properly. Suppress warning for now.
pub fn touch_loc_and_then_intercept(
    ctx: &mut Context,
    touch_loc: Point2<f32>,
) -> Result<RoutePlanInfo, RoutePlanError> {
    if NotOnFlatGround.evaluate(ctx) {
        return Err(RoutePlanError::MustBeOnFlatGround);
    }
    if IsSkidding.evaluate(ctx) {
        return Err(RoutePlanError::MustNotBeSkidding);
    }

    let me = ctx.me();

    // Naively guess the intercept location by driving to the corner and then
    // trying to intercept.
    let naive_touch = simple_drive_towards(&me.into(), touch_loc)?;
    let guess = naive_ground_intercept(
        ctx.scenario
            .ball_prediction()
            .iter()
            .skip_while(|f| f.t < naive_touch.duration()),
        naive_touch.end().loc,
        naive_touch.end().vel,
        naive_touch.end().boost,
        |ball| ball.loc.z < GroundedHit::max_ball_z() && ball.vel.z < 25.0,
    );
    let guess = some_or_else!(guess, {
        return Err(RoutePlanError::UnknownIntercept);
    });

    let a: RoutePlan = powerslide_turn(&me.into(), touch_loc, guess.ball_loc.to_2d())?;
    let b: RoutePlan = simple_drive_towards(&a.end(), guess.ball_loc.to_2d())?;
    Ok(RoutePlanInfo {
        plan: RoutePlan::new(
            a.segments
                .into_iter()
                .chain(b.segments.into_iter())
                .collect(),
        ),
        intercept: guess,
    })
}

fn powerslide_turn(
    start: &CarState,
    touch_loc: Point2<f32>,
    turn_point_loc: Point2<f32>,
) -> Result<RoutePlan, RoutePlanError> {
    let approach = simple_drive_towards(start, touch_loc)?;
    let approach_duration = approach.duration() - 1.0;
    let approach = approach.truncate_to_duration(approach_duration).unwrap();

    let turn_target_rot = CAR_LOCAL_FORWARD_AXIS_2D
        .as_ref()
        .rotation_to(turn_point_loc - touch_loc);
    let turn_plan = CarPowerslideTurn::evaluate(
        approach.end().loc.to_2d(),
        approach.end().vel.to_2d(),
        1.0,
        car_forward_axis_2d(turn_target_rot),
    )
    .ok_or_else(|| RoutePlanError::OtherError("CarPowerslideTurn returned None"))?;

    let turn = PowerslideTurn::new(turn_plan, start.boost);
    println!("slide duration: {:?}", turn.duration());
    Ok(RoutePlan::new(
        approach
            .segments
            .into_iter()
            .chain(iter::once::<Box<SegmentPlan>>(Box::new(turn)))
            .collect(),
    ))
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
        start.boost,
        target_loc,
    ));

    Ok(RoutePlan::new(
        turn.into_iter()
            .flat_map(|p| p.segments.into_iter())
            .chain(iter::once::<Box<SegmentPlan>>(straight))
            .collect(),
    ))
}

fn turn_towards(
    start: &CarState,
    target_loc: Point2<f32>,
) -> Result<Option<RoutePlan>, RoutePlanError> {
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

    let segment = SimpleArc::new(
        turn_center,
        turn_radius,
        start_loc,
        start_vel,
        start.boost,
        tangent,
    )
    .map_err(|err| RoutePlanError::OtherError(err.to_str()))?;
    Ok(Some(RoutePlan::new(vec![Box::new(segment)])))
}
