use crate::{
    behavior::{Action, Behavior, Priority},
    eeg::{color, Drawable},
    maneuvers::{GroundedHit, GroundedHitAimContext, GroundedHitTarget, GroundedHitTargetAdjust},
    plan::hit_angle::{feasible_hit_angle_away, feasible_hit_angle_toward},
    routing::{behavior::FollowRoute, plan::GroundIntercept},
    strategy::Context,
    utils::{Wall, WallRayCalculator},
};
use common::prelude::*;
use nalgebra::Point2;
use std::f32::consts::PI;

pub struct TepidHit;

impl TepidHit {
    pub fn new() -> Self {
        TepidHit
    }
}

impl Behavior for TepidHit {
    fn name(&self) -> &str {
        stringify!(TepidHit)
    }

    fn execute2(&mut self, _ctx: &mut Context) -> Action {
        return Action::call(chain!(
            Priority::Idle,
            [
                FollowRoute::new(GroundIntercept::new()),
                GroundedHit::hit_towards(time_wasting_hit),
            ],
        ));
    }
}

fn time_wasting_hit(ctx: &mut GroundedHitAimContext) -> Result<GroundedHitTarget, ()> {
    let me_loc = ctx.car.Physics.loc_2d();
    let ball_loc = ctx.intercept_ball_loc.to_2d();
    let offense_aim = ctx.game.enemy_back_wall_center();
    let defense_avoid = ctx.game.own_back_wall_center();

    let naive_offense = (ball_loc - me_loc).rotation_to(offense_aim - me_loc);
    let naive_defense = (ball_loc - me_loc).rotation_to(defense_avoid - me_loc);

    let aim_loc = if naive_offense.angle().abs() < naive_defense.angle().abs() {
        ctx.eeg
            .draw(Drawable::print("toward enemy goal", color::GREEN));
        feasible_hit_angle_toward(ball_loc, me_loc, offense_aim, PI / 6.0)
    } else {
        ctx.eeg
            .draw(Drawable::print("away from own goal", color::GREEN));
        feasible_hit_angle_away(ball_loc, me_loc, defense_avoid, PI / 6.0)
    };

    let aim_loc = Point2::from(WallRayCalculator::calculate(ball_loc, aim_loc));

    match WallRayCalculator::wall_for_point(ctx.game, aim_loc) {
        Wall::OwnGoal => {
            ctx.eeg.log("[TepidHit] refusing to own goal");
            return Err(());
        }
        _ => {}
    }

    Ok(GroundedHitTarget::new(
        ctx.intercept_time,
        GroundedHitTargetAdjust::RoughAim,
        aim_loc,
    )
    .dodge(!is_chippable(ctx, aim_loc)))
}

fn is_chippable(ctx: &mut GroundedHitAimContext, aim_loc: Point2<f32>) -> bool {
    let shot_angle = ctx
        .car
        .Physics
        .forward_axis_2d()
        .rotation_to(&(aim_loc - ctx.car.Physics.loc_2d()).to_axis())
        .angle()
        .abs();

    let goalward_angle = (ctx.intercept_ball_loc.to_2d() - ctx.car.Physics.loc_2d())
        .rotation_to(ctx.game.enemy_goal().center_2d - ctx.intercept_ball_loc.to_2d())
        .angle()
        .abs();

    // Target a pretty specific scenario in the enemy corner, where you roll the
    // ball around the side wall without jumping so you can quickly recover and dish
    // it in.
    ctx.intercept_ball_loc.x.abs() >= 3000.0
        && (ctx.game.enemy_goal().center_2d.y - ctx.intercept_ball_loc.y).abs() < 2000.0
        && ctx.intercept_ball_loc.z < 130.0
        && shot_angle < PI / 4.0
        && goalward_angle < PI / 2.0
}
