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
    strategy::Pitch,
    utils::geometry::{ExtendF32, Plane},
};
use derive_new::new;
use nalgebra::Point3;
use nameof::name_of_type;
use vec_box::vec_box;

#[derive(Clone, new)]
pub struct WallIntercept;

impl RoutePlanner for WallIntercept {
    fn name(&self) -> &'static str {
        name_of_type!(WallIntercept)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);

        let mut balls = ctx.ball_prediction.iter();
        while let Some(ball) = balls.next() {
            match try_intercept(ctx, ball) {
                Ok(plan) => return Ok(plan),
                Err(Skip::No) => continue,
                Err(Skip::Yes) => {
                    // Skip ahead to try to stay within the frame budget.
                    skip_time(&mut balls, 0.125);
                    continue;
                }
            }
        }
        Err(RoutePlanError::UnknownIntercept)
    }
}

fn try_intercept(ctx: &PlanningContext, ball: &BallFrame) -> Result<RoutePlan, Skip> {
    if which_wall(ctx.game.pitch(), ball.loc).is_none() {
        return Err(Skip::No);
    }

    let turn = WallTurnPlanner::new(ball.loc);
    let straight = WallStraightPlanner::new(ball.loc);
    let planner = ChainedPlanner::chain(vec_box![turn, straight]);

    let plan = match PlanningContext::plan_2(&planner, ctx) {
        Ok((plan, _log)) => plan,
        Err(_) => return Err(Skip::Yes),
    };
    let tail = match plan.provisional_expand_2(ctx.game, ctx.ball_prediction) {
        Ok(tail) => tail,
        Err(_) => return Err(Skip::Yes),
    };
    let expansion = ProvisionalPlanExpansion::new(&*plan.segment, &tail);

    if expansion.duration() < ball.t {
        Err(Skip::Yes)
    } else {
        Ok(plan)
    }
}

enum Skip {
    No,
    Yes,
}

fn which_wall(pitch: &Pitch, loc: Point3<f32>) -> Option<&Plane> {
    let wall = pitch.closest_plane(&loc);
    if wall.normal.z != 0.0 {
        // Must be a wall, not ceiling or floor.
        return None;
    }
    if wall.distance_to_point(&loc) >= 500.0 {
        return None;
    }
    Some(wall)
}

fn skip_time<'a>(xs: &mut impl Iterator<Item = &'a BallFrame>, time: f32) {
    let x = some_or_else!(xs.next(), {
        return;
    });
    let num = (time / x.dt()).into_almost_int();
    assert!(num >= 1);
    for _ in xs.take(num as usize - 1) {}
}
