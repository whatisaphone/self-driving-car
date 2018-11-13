use behavior::{Action, Behavior, Priority};
use common::prelude::*;
use eeg::{color, Drawable};
use maneuvers::GroundedHit;
use nalgebra::{Point2, Point3};
use plan::hit_angle::{feasible_hit_angle_away, feasible_hit_angle_toward};
use routing::{behavior::FollowRoute, plan::GroundIntercept};
use std::f32::consts::PI;
use strategy::Context;
use utils::WallRayCalculator;

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

fn time_wasting_hit(ctx: &mut Context, intercept_ball_loc: Point3<f32>) -> Result<Point2<f32>, ()> {
    let me_loc = ctx.me().Physics.loc_2d();
    let ball_loc = intercept_ball_loc.to_2d();
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

    Ok(Point2::from(WallRayCalculator::calculate(
        ball_loc, aim_loc,
    )))
}
