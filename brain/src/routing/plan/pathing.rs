use crate::routing::{
    models::{CarState, RoutePlanner},
    plan::{
        ground_straight::GroundStraightPlanner, ground_turn::PathingUnawareTurnPlanner,
        higher_order::ChainedPlanner,
    },
    segments::StraightMode,
};
use common::{physics, prelude::*, rl};
use nalgebra::Point2;

/// Calculate whether driving straight to `target_loc` would intersect the goal
/// wall. If so, return the route we should follow to get outside the goal.
pub fn avoid_plowing_into_goal_wall(
    start: &CarState,
    target_loc: Point2<f32>,
) -> Option<Box<RoutePlanner>> {
    match avoid_plowing_into_goal_wall_waypoint(start, target_loc) {
        None => None,
        Some(waypoint) => Some(ChainedPlanner::chain(vec![
            Box::new(PathingUnawareTurnPlanner::new(waypoint, None)),
            Box::new(GroundStraightPlanner::new(
                waypoint,
                None,
                0.0,
                StraightMode::Asap,
            )),
        ])),
    }
}

/// Calculate whether driving straight to `target_loc` would intersect the goal
/// wall. If so, return the waypoint we should drive to first to avoid
/// embarrassing ourselves.
fn avoid_plowing_into_goal_wall_waypoint(
    start: &CarState,
    target_loc: Point2<f32>,
) -> Option<Point2<f32>> {
    let margin = 125.0;

    // Only proceed if we're crossing over the goalline.
    let brink = rl::FIELD_MAX_Y * start.loc.y.signum();
    if (brink - start.loc.y).signum() == (brink - target_loc.y).signum() {
        return None;
    }

    let goal_y = rl::FIELD_MAX_Y * start.loc.y.signum();
    let ray = physics::car_forward_axis_2d(start.rot.to_2d());
    let toi = (goal_y - start.loc.y) / ray.y;
    let cross_x = start.loc.x + toi * ray.x;
    if cross_x.abs() >= rl::GOALPOST_X - margin {
        Some(Point2::new(
            (rl::GOALPOST_X - margin) * cross_x.signum(),
            (rl::FIELD_MAX_Y - margin) * start.loc.y.signum(),
        ))
    } else {
        None
    }
}
