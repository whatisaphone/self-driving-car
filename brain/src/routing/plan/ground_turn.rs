use crate::{
    routing::{
        models::{
            CarState, PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner,
        },
        plan::{
            ground_powerslide::GroundSimplePowerslideTurn, higher_order::ChainedPlanner, pathing,
        },
        recover::{IsSkidding, NotOnFlatGround},
        segments::{NullSegment, SimpleArc, Turn},
    },
    utils::geometry::circle_point_tangents,
};
use chip;
use common::{prelude::*, rl};
use derive_new::new;
use nalgebra::Point2;
use simulate::linear_interpolate;
use std::f32::consts::PI;

const SLOWEST_TURNING_SPEED: f32 = 900.0;

#[derive(Clone, new)]
pub struct TurnPlanner {
    target_face: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for TurnPlanner {
    fn name(&self) -> &'static str {
        stringify!(TurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_face", self.target_face);

        let pathing_unaware_planner = PathingUnawareTurnPlanner::new(self.target_face, None);
        let turn = pathing_unaware_planner.plan(ctx, dump)?;
        dump.log_plan(self, &turn);
        let plan =
            match pathing::avoid_plowing_into_goal_wall(&turn.segment.end(), self.target_face) {
                None => turn,
                Some(divert) => {
                    dump.log(self, "diverting due to avoid_plowing_into_goal_wall");
                    ChainedPlanner::new(divert, Some(Box::new(pathing_unaware_planner)))
                        .plan(ctx, dump)?
                }
            };
        Ok(ChainedPlanner::join_planner(plan, self.next.clone()))
    }
}

#[derive(Clone, new)]
pub struct PathingUnawareTurnPlanner {
    target_face: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for PathingUnawareTurnPlanner {
    fn name(&self) -> &'static str {
        stringify!(PathingUnawareTurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_face", self.target_face);

        if self.should_powerslide(&ctx.start) {
            let turn = GroundSimplePowerslideTurn::new(self.target_face);
            ChainedPlanner::new(Box::new(turn), self.next.clone()).plan(ctx, dump)
        } else {
            SimpleTurnPlanner::new(self.target_face, self.next.clone()).plan(ctx, dump)
        }
    }
}

impl PathingUnawareTurnPlanner {
    fn should_powerslide(&self, start: &CarState) -> bool {
        if start.vel.to_2d().norm() < 1000.0 {
            return false;
        }

        let munged_start_loc = start.loc.to_2d() + start.vel.to_2d() * 0.25;
        let turn = start
            .forward_axis_2d()
            .rotation_to(&(self.target_face - munged_start_loc).to_axis());
        let angle_cutoff = linear_interpolate(
            &[0.0, rl::CAR_NORMAL_SPEED],
            &[PI * 0.25, PI * 0.50],
            start.vel.to_2d().norm(),
        );
        turn.angle().abs() > angle_cutoff
    }
}

#[derive(Clone, new)]
pub struct SimpleTurnPlanner {
    target_loc: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for SimpleTurnPlanner {
    fn name(&self) -> &'static str {
        stringify!(SimpleTurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        let turn_radius =
            1.0 / chip::max_curvature(ctx.start.vel.norm().max(SLOWEST_TURNING_SPEED));
        let turn = match calculate_circle_turn(&ctx.start, turn_radius, self.target_loc)? {
            Some(x) => x,
            None => {
                return Ok(RoutePlan {
                    segment: Box::new(NullSegment::new(ctx.start.clone())),
                    next: self.next.clone(),
                });
            }
        };
        let segment = Turn::new(
            ctx.start.clone(),
            self.target_loc,
            turn.center,
            turn.radius,
            turn.tangent,
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: self.next.clone(),
        })
    }
}

#[allow(dead_code)] // This basically works but I can't make use of it just yet.
#[derive(Clone, new)]
struct ArcTowards {
    target_loc: Point2<f32>,
    next: Option<Box<RoutePlanner>>,
}

#[allow(dead_code)]
impl RoutePlanner for ArcTowards {
    fn name(&self) -> &'static str {
        stringify!(ArcTowards)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });

        let turn_radius =
            1.0 / chip::max_curvature(ctx.start.vel.norm().max(SLOWEST_TURNING_SPEED));
        let turn = match calculate_circle_turn(&ctx.start, turn_radius, self.target_loc)? {
            Some(x) => x,
            None => {
                return Ok(RoutePlan {
                    segment: Box::new(NullSegment::new(ctx.start.clone())),
                    next: self.next.clone(),
                });
            }
        };

        let segment = SimpleArc::new(
            turn.center,
            turn.radius,
            ctx.start.loc.to_2d(),
            ctx.start.vel.to_2d(),
            ctx.start.boost,
            turn.tangent,
        )
        .map_err(|err| RoutePlanError::OtherError(err.to_str()))?;
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: self.next.clone(),
        })
    }
}

fn calculate_circle_turn(
    start: &CarState,
    turn_radius: f32,
    target_loc: Point2<f32>,
) -> Result<Option<CircleTurn>, RoutePlanError> {
    let start_loc = start.loc.to_2d();
    let start_forward_axis = start.forward_axis_2d();
    let start_right_axis = start.right_axis_2d();

    // Check if we're already facing the target
    let turn_rot = start_forward_axis.rotation_to(&(target_loc - start_loc).to_axis());
    if turn_rot.angle().abs() < 2.0_f32.to_radians() {
        return Ok(None);
    }

    // Define a circle where our current location/rotation form a tangent.
    let turn_center =
        start_loc + start_right_axis.as_ref() * turn_rot.angle().signum() * turn_radius;

    // Figure out which tangent point is relevant for this route.
    let [tangent1, tangent2] = match circle_point_tangents(turn_center, turn_radius, target_loc) {
        Some(x) => x,
        None => {
            return Err(RoutePlanError::TurningRadiusTooTight);
        }
    };
    let t1_rot = (start_loc - turn_center).rotation_to(tangent1 - turn_center);
    let t2_rot = (start_loc - turn_center).rotation_to(tangent2 - turn_center);
    let t1_forward_axis = t1_rot * start_forward_axis;
    let t2_forward_axis = t2_rot * start_forward_axis;
    let t1_dot = t1_forward_axis.dot(&(target_loc - tangent1));
    let t2_dot = t2_forward_axis.dot(&(target_loc - tangent2));
    let tangent = match (t1_dot > 0.0, t2_dot > 0.0) {
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
