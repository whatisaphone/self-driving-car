use behavior::{Behavior, Chain, Defense, Offense, Priority};
use maneuvers::FiftyFifty;
use std::{f32::consts::PI, iter::once};
use strategy::{scenario::Wall, Context};
use utils::{my_goal_center_2d, ExtendF32, ExtendPhysics, ExtendVector2, ExtendVector3};

const POSSESSION_CONTESTABLE: f32 = 0.5;

pub fn baseline(ctx: &mut Context) -> Box<Behavior> {
    match ctx.scenario.push_wall() {
        Wall::OwnGoal | Wall::OwnBackWall => Box::new(Defense::new()),
        _ => Box::new(Offense::new()),
    }
}

pub fn override_(ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>> {
    if current.priority() < Priority::Save
        && ctx.scenario.possession().abs() < POSSESSION_CONTESTABLE
        && enemy_can_shoot(ctx)
    {
        return Some(Box::new(Chain::new(
            Priority::Save,
            vec![Box::new(FiftyFifty::new())],
        )));
    }

    None
}

fn enemy_can_shoot(ctx: &mut Context) -> bool {
    let ball_loc = ctx.scenario.enemy_intercept().1.to_2d();
    let angle_car_ball = ctx.enemy().Physics.loc().to_2d().angle_to(ball_loc);
    let angle_ball_goal = ball_loc.angle_to(my_goal_center_2d());
    let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
    angle_diff < PI / 3.0
}
