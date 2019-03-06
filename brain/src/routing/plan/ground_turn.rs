use crate::{
    routing::{
        models::{
            CarState, CarState2D, PlanningContext, PlanningDump, RoutePlan, RoutePlanError,
            RoutePlanner,
        },
        plan::{
            ground_powerslide::GroundSimplePowerslideTurn, higher_order::ChainedPlanner, pathing,
        },
        recover::{IsSkidding, NotOnFlatGround},
        segments::{NullSegment, SimpleArc, Turn},
    },
    utils::geometry::{circle_point_tangents, flattener::Flattener},
};
use common::{physics::CAR_LOCAL_FORWARD_AXIS_2D, prelude::*, rl};
use derive_new::new;
use nalgebra::{Point2, Unit, Vector2};
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;
use vec_box::vec_box;

const SLOWEST_TURNING_SPEED: f32 = 900.0;

#[derive(Clone)]
pub struct TurnPlanner {
    target_face: Point2<f32>,
    next: Option<Box<dyn RoutePlanner>>,
    reverse_angle_hint: Option<Unit<Vector2<f32>>>,
}

impl TurnPlanner {
    pub fn new(target_face: Point2<f32>, next: Option<Box<dyn RoutePlanner>>) -> Self {
        Self {
            target_face,
            next,
            reverse_angle_hint: None,
        }
    }

    pub fn reverse_angle_hint(mut self, reverse_angle_hint: Unit<Vector2<f32>>) -> Self {
        self.reverse_angle_hint = Some(reverse_angle_hint);
        self
    }
}

impl RoutePlanner for TurnPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(TurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_face", self.target_face);

        let pathing_unaware_planner =
            PathingUnawareTurnPlanner::new(self.target_face, self.reverse_angle_hint);
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
    reverse_angle_hint: Option<Unit<Vector2<f32>>>,
}

impl RoutePlanner for PathingUnawareTurnPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(PathingUnawareTurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_face", self.target_face);

        if self.should_powerslide(&ctx.start) {
            self.powerslide_with_angle_hint_hack(ctx, dump)
        } else {
            SimpleTurnPlanner::new(self.target_face, None).plan(ctx, dump)
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
            .angle_to(&(self.target_face - munged_start_loc).to_axis());
        let angle_cutoff = linear_interpolate(
            &[0.0, rl::CAR_NORMAL_SPEED],
            &[PI / 4.0, PI / 3.0],
            start.vel.to_2d().norm(),
        );
        turn.abs() > angle_cutoff
    }

    fn powerslide_with_angle_hint_hack(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        let reverse_angle_hint = some_or_else!(self.reverse_angle_hint, {
            // No hack necessary.
            return GroundSimplePowerslideTurn::new(self.target_face).plan(ctx, dump);
        });

        let car_forward_axis = ctx.start.forward_axis_2d();
        let car_to_face = self.target_face - ctx.start.loc_2d();

        if ctx.start.loc.x.abs() >= ctx.game.field_max_x() - 750.0 {
            // Too close to wall.
            return GroundSimplePowerslideTurn::new(self.target_face).plan(ctx, dump);
        }

        let naive_rot = car_forward_axis.rotation_to(&car_to_face.to_axis());
        if naive_rot.angle().abs() < 135.0_f32.to_radians() {
            // No hack necessary.
            return GroundSimplePowerslideTurn::new(self.target_face).plan(ctx, dump);
        }

        // First, turn the long way around.
        let long_way_around = ctx.start.loc_2d()
            + car_forward_axis.rotation_to(&reverse_angle_hint)
                * (CAR_LOCAL_FORWARD_AXIS_2D.into_inner() * 1000.0);
        let step1 = GroundSimplePowerslideTurn::new(long_way_around);
        // Then turn the rest of the way.
        let step2 = GroundSimplePowerslideTurn::new(self.target_face);
        ChainedPlanner::chain(vec_box![step1, step2]).plan(ctx, dump)
    }
}

#[derive(Clone, new)]
pub struct SimpleTurnPlanner {
    target_loc: Point2<f32>,
    next: Option<Box<dyn RoutePlanner>>,
}

impl RoutePlanner for SimpleTurnPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(SimpleTurnPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        let start = ctx.start.flatten(&Flattener::identity());

        let turn_radius =
            1.0 / chip::max_curvature(ctx.start.vel.norm().max(SLOWEST_TURNING_SPEED));
        let turn = match calculate_circle_turn(&start, turn_radius, self.target_loc)? {
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
    next: Option<Box<dyn RoutePlanner>>,
}

impl RoutePlanner for ArcTowards {
    fn name(&self) -> &'static str {
        name_of_type!(ArcTowards)
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
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });

        let start = ctx.start.flatten(&Flattener::identity());

        let turn_radius =
            1.0 / chip::max_curvature(ctx.start.vel.norm().max(SLOWEST_TURNING_SPEED));
        let turn = match calculate_circle_turn(&start, turn_radius, self.target_loc)? {
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

pub fn calculate_circle_turn(
    start: &CarState2D,
    turn_radius: f32,
    target_loc: Point2<f32>,
) -> Result<Option<CircleTurn>, RoutePlanError> {
    let start_loc = start.loc;
    let start_forward_axis = start.forward_axis();
    let start_right_axis = start.right_axis();

    // Check if we're already facing the target
    let turn_angle = start_forward_axis.angle_to(&(target_loc - start_loc).to_axis());
    if turn_angle.abs() < 2.0_f32.to_radians() {
        return Ok(None);
    }

    // Define a circle where our current location/rotation form a tangent.
    let turn_center = start_loc + start_right_axis.as_ref() * turn_angle.signum() * turn_radius;

    // Figure out which tangent point is relevant for this route.
    let [tangent1, tangent2] = match circle_point_tangents(turn_center, turn_radius, target_loc) {
        Some(x) => x,
        None => {
            return Err(RoutePlanError::TurningRadiusTooTight);
        }
    };
    let t1_rot = (start_loc - turn_center).rotation_to(&(tangent1 - turn_center));
    let t2_rot = (start_loc - turn_center).rotation_to(&(tangent2 - turn_center));
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
        start_loc,
        radius: turn_radius,
        tangent,
    }))
}

pub struct CircleTurn {
    pub center: Point2<f32>,
    pub radius: f32,
    /// The point on the circle where the turn starts.
    pub start_loc: Point2<f32>,
    /// The point on the circle where the turn ends.
    pub tangent: Point2<f32>,
}

impl CircleTurn {
    pub fn sweep(&self) -> f32 {
        // Go the long way around the circle (more than 180°) if necessary. This avoids
        // an impossible route with discontinuous reversals at each tangent.
        (self.start_loc - self.center).angle_to(&(self.tangent - self.center))
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    #[ignore("this is a demo, not a test")]
    fn powerslide_angle_hint_hack() {
        TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-3630.1199, 2566.14, 92.71),
                ball_vel: Vector3::new(269.04102, -364.501, 0.0),
                ball_ang_vel: Vector3::new(3.99451, 2.94831, -3.26771),
                car_loc: Point3::new(-3080.8298, 1091.5399, 85.61),
                car_rot: Rotation3::from_unreal_angles(0.584991, -1.3567058, 0.2882568),
                car_vel: Vector3::new(-81.620995, -1268.0409, -183.151),
                car_ang_vel: Vector3::new(2.51891, -0.12501, -0.42931),
                ..Default::default()
            })
            .starting_boost(60.0)
            .soccar()
            .run_for_millis(5000);
    }
}
