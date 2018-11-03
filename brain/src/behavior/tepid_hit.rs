use behavior::{Action, Behavior, Priority};
use common::prelude::*;
use eeg::{color, Drawable};
use maneuvers::GroundedHit;
use nalgebra::{Point2, Point3, UnitComplex, Vector2};
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
    let offense_aim = ctx.game.enemy_goal().center_2d - Point2::origin();
    let defense_aim = ctx.game.own_goal().center_2d - Point2::origin();

    let naive_offense = (ball_loc - me_loc).rotation_to(offense_aim);
    let naive_defense = (ball_loc - me_loc).rotation_to(defense_aim);

    let ideal = if naive_offense.angle().abs() < naive_defense.angle().abs() {
        ctx.eeg
            .draw(Drawable::print("toward enemy goal", color::GREEN));
        offense_aim
    } else {
        ctx.eeg
            .draw(Drawable::print("away from own goal", color::GREEN));
        defense_aim
    };

    let ideal_adjust = (ball_loc - me_loc).rotation_to(ideal).angle();
    let adjust = UnitComplex::new(ideal_adjust.max(-PI / 6.0).min(PI / 6.0));
    let rot = Vector2::x().rotation_to(ball_loc - me_loc) * adjust;

    Ok(Point2::from_coordinates(WallRayCalculator::calc_ray(
        ball_loc,
        rot.angle(),
    )))
}
