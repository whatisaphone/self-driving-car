use crate::{
    plan::ball::BallFrame,
    routing::{
        models::{
            PlanningContext, PlanningDump, ProvisionalPlanExpansion, RoutePlan, RoutePlanError,
            RoutePlanner,
        },
        plan::{
            higher_order::ChainedPlanner, wall_straight::WallStraightPlanner,
            wall_turn::WallTurnPlanner,
        },
    },
    strategy::{Context2, Game, Pitch},
    utils::geometry::{ExtendF32, Plane},
};
use common::{prelude::*, Time};
use nalgebra::Point3;
use nameof::name_of_type;
use vec_box::vec_box;

#[derive(Clone)]
pub struct WallIntercept {
    must_be_wall: bool,
    must_be_side_wall: bool,
    must_be_vanilla_safe_offensive: bool,
}

impl WallIntercept {
    pub fn new() -> Self {
        Self {
            must_be_wall: false,
            must_be_side_wall: false,
            must_be_vanilla_safe_offensive: true,
        }
    }

    pub fn must_be_wall(mut self, must_be_wall: bool) -> Self {
        self.must_be_wall = must_be_wall;
        self
    }

    pub fn must_be_side_wall(mut self, must_be_side_wall: bool) -> Self {
        self.must_be_side_wall = must_be_side_wall;
        self
    }
}

impl RoutePlanner for WallIntercept {
    fn name(&self) -> &'static str {
        name_of_type!(WallIntercept)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        let (ball, plan) = self
            .calc_plan(ctx)
            .map_err(|_| RoutePlanError::NoWallIntercept)?;
        dump.log_pretty(self, "intercept ball time", Time(ball.t));
        dump.log_pretty(self, "intercept ball loc", ball.loc);
        Ok(plan)
    }
}

impl WallIntercept {
    pub fn calc_intercept<'ball>(
        &self,
        ctx: &Context2<'_, 'ball>,
    ) -> Result<&'ball BallFrame, &'static str> {
        match self.calc_plan(&PlanningContext::from_context(ctx)) {
            Ok((ball, _plan)) => Ok(ball),
            Err(reason) => Err(reason),
        }
    }

    fn calc_plan<'ball>(
        &self,
        ctx: &PlanningContext<'_, 'ball>,
    ) -> Result<(&'ball BallFrame, RoutePlan), &'static str> {
        let mut balls = ctx.ball_prediction.iter();
        let mut fail_reason = None;
        while let Some(ball) = balls.next() {
            match self.eval_intercept(ctx, ball) {
                Ok(plan) => return Ok((ball, plan)),
                Err((skip, reason)) => {
                    if skip == Skip::Yes {
                        // Whatever just happened was computationally expensive, so skip ahead to
                        // try to stay within the frame budget.
                        skip_time(&mut balls, 0.125);
                    }
                    if fail_reason.is_none() {
                        if let Some(reason) = reason {
                            fail_reason = Some(reason);
                        }
                    }
                    continue;
                }
            }
        }
        Err(fail_reason.unwrap_or("unknown intercept"))
    }

    fn eval_intercept(
        &self,
        ctx: &PlanningContext<'_, '_>,
        ball: &BallFrame,
    ) -> Result<RoutePlan, (Skip, Option<&'static str>)> {
        if self.eligible_wall(ctx.game.pitch(), ball.loc).is_none() {
            return Err((Skip::No, None));
        }

        match self.check_conditions(ctx.game, ball.loc, ctx.start.loc) {
            Ok(()) => {}
            Err(reason) => return Err((Skip::No, Some(reason))),
        }

        let turn = WallTurnPlanner::new(ball.loc);
        let straight = WallStraightPlanner::new(ball.loc);
        let planner = ChainedPlanner::chain(vec_box![turn, straight]);

        let plan = match PlanningContext::plan_2(&planner, ctx) {
            Ok((plan, _log)) => plan,
            Err(_) => return Err((Skip::Yes, None)),
        };
        let tail = match plan.provisional_expand_2(ctx.game, ctx.ball_prediction) {
            Ok(tail) => tail,
            Err(_) => return Err((Skip::Yes, None)),
        };
        let expansion = ProvisionalPlanExpansion::new(&*plan.segment, &tail);

        if expansion.duration() >= ball.t {
            return Err((Skip::Yes, None));
        }

        Ok(plan)
    }

    fn eligible_wall<'pitch>(
        &self,
        pitch: &'pitch Pitch,
        loc: Point3<f32>,
    ) -> Option<&'pitch Plane> {
        let plane = pitch.closest_plane(&loc);
        if self.must_be_wall && plane.normal.z != 0.0 {
            return None;
        }
        if self.must_be_side_wall && plane.normal.x.abs() != 1.0 {
            return None;
        }
        if plane.distance_to_point(&loc) >= 300.0 {
            return None;
        }
        Some(plane)
    }

    fn check_conditions(
        &self,
        game: &Game<'_>,
        intercept_loc: Point3<f32>,
        car_loc: Point3<f32>,
    ) -> Result<(), &'static str> {
        if self.must_be_vanilla_safe_offensive {
            let push_angle = (intercept_loc.to_2d() - car_loc.to_2d())
                .angle_to(&(game.enemy_goal().center_2d - game.own_goal().center_2d));
            if push_angle >= 75.0_f32.to_radians() {
                return Err("push_angle is not good enough");
            }
        }

        // If the ball is in the corner on the wall, there's a big risk of missing due
        // to the corner curve or inaccurate ball prediction. If the ball is in the
        // corner on the ground, we're probably descending from the wall and we should
        // proceed anyway.
        let plane = game.pitch().closest_plane(&intercept_loc);
        let is_wall = plane.normal.z == 0.0;
        let ball_in_corner = (intercept_loc.y - game.own_goal().center_2d.y).abs() < 2000.0
            || (intercept_loc.y - game.enemy_goal().center_2d.y).abs() < 1250.0;
        if is_wall && ball_in_corner {
            return Err("too close to the corner");
        }

        Ok(())
    }
}

#[derive(Eq, PartialEq)]
enum Skip {
    No,
    Yes,
}

fn skip_time<'a>(xs: &mut impl Iterator<Item = &'a BallFrame>, time: f32) {
    let x = some_or_else!(xs.next(), {
        return;
    });
    let num = (time / x.dt()).into_almost_int();
    assert!(num >= 1);
    for _ in xs.take(num as usize - 1) {}
}
