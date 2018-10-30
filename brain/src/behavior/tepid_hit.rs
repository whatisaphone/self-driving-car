use behavior::{Action, Behavior, Priority};
use common::prelude::*;
use eeg::{color, Drawable};
use maneuvers::GroundedHit;
use nalgebra::{Point2, Point3};
use plan::hit_angle::{feasible_hit_angle_away, feasible_hit_angle_toward};
use routing::{behavior::FollowRoute, plan::GroundIntercept};
use std::f32::consts::PI;
use strategy::Context;
use utils::{
    enemy_goal_center, my_goal_center_2d, ExtendF32, ExtendVector2, ExtendVector3,
    WallRayCalculator,
};

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
    let me = ctx.me();
    let ball = intercept_ball_loc.coords.to_2d();
    let goal = enemy_goal_center();
    let avoid = my_goal_center_2d();

    let angle_ball = me.Physics.loc().to_2d().angle_to(ball);
    let angle_forward = me.Physics.loc().to_2d().angle_to(enemy_goal_center());
    let angle_backward = me.Physics.loc().to_2d().angle_to(my_goal_center_2d());

    let angle_offense = (angle_ball - angle_forward).normalize_angle().abs();
    let angle_defense = (angle_ball - angle_backward).normalize_angle().abs();

    let theta = if angle_offense < angle_defense {
        ctx.eeg
            .draw(Drawable::print("toward enemy goal", color::GREEN));
        feasible_hit_angle_toward(ball, me.Physics.loc().to_2d(), goal, PI / 6.0)
    } else {
        ctx.eeg
            .draw(Drawable::print("away from own goal", color::GREEN));
        feasible_hit_angle_away(ball, me.Physics.loc().to_2d(), avoid, PI / 6.0)
    };

    Ok(Point2::from_coordinates(WallRayCalculator::calc_ray(
        ball, theta,
    )))
}
