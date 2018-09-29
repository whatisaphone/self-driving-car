use behavior::{Behavior, Chain, Defense, Offense, Priority};
use maneuvers::{FiftyFifty, GetToFlatGround};
use std::f32::consts::PI;
use strategy::{scenario::Scenario, Context};
use utils::{my_goal_center_2d, ExtendF32, ExtendPhysics, ExtendVector2, ExtendVector3, Wall};

pub fn baseline(ctx: &mut Context) -> Box<Behavior> {
    if !GetToFlatGround::on_flat_ground(ctx.packet) {
        return Box::new(GetToFlatGround::new());
    }

    match ctx.scenario.push_wall() {
        Some(Wall::OwnGoal) | Some(Wall::OwnBackWall) | None => Box::new(Defense::new()),
        Some(_) => Box::new(Offense::new()),
    }
}

pub fn override_(ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>> {
    if current.priority() < Priority::Save && enemy_can_shoot(ctx) {
        if ctx.scenario.possession().abs() < Scenario::POSSESSION_CONTESTABLE {
            ctx.eeg.log(format!(
                "enemy can shoot, possession = {:.2}, going for 50/50",
                ctx.scenario.possession()
            ));
            return Some(Box::new(Chain::new(
                Priority::Save,
                vec![Box::new(FiftyFifty::new())],
            )));
        }

        if ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE {
            ctx.eeg.log(format!(
                "enemy can shoot, possession = {:.2}, going to defense",
                ctx.scenario.possession()
            ));
            return Some(Box::new(Chain::new(
                Priority::Save,
                vec![Box::new(Defense::new())],
            )));
        }
    }

    None
}

fn enemy_can_shoot(ctx: &mut Context) -> bool {
    let enemy_intercept = match ctx.scenario.enemy_intercept() {
        Some(i) => i,
        None => return false,
    };
    let ball_loc = enemy_intercept.1.to_2d();
    if (ball_loc - my_goal_center_2d()).norm() >= 3000.0 {
        return false;
    }
    let angle_car_ball = ctx.enemy().Physics.loc().to_2d().angle_to(ball_loc);
    let angle_ball_goal = ball_loc.angle_to(my_goal_center_2d());
    let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
    angle_diff < PI / 3.0
}
