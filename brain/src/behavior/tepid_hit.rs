use behavior::{Action, Behavior};
use maneuvers::BounceShot;
use nalgebra::Vector3;
use plan::hit_angle::{feasible_hit_angle_away, feasible_hit_angle_toward};
use std::f32::consts::PI;
use strategy::Context;
use utils::{
    enemy_goal_center, my_goal_center_2d, ExtendF32, ExtendPhysics, ExtendVector2, ExtendVector3,
    WallRayCalculator,
};

pub struct TepidHit {
    intercept_loc: Vector3<f32>,
}

impl TepidHit {
    pub fn new(intercept_loc: Vector3<f32>) -> Self {
        Self { intercept_loc }
    }
}

impl Behavior for TepidHit {
    fn name(&self) -> &str {
        stringify!(TepidHit)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();
        let ball = self.intercept_loc.to_2d();
        let goal = enemy_goal_center();
        let avoid = my_goal_center_2d();

        let angle_ball = me.Physics.loc().to_2d().angle_to(ball);
        let angle_forward = me.Physics.loc().to_2d().angle_to(enemy_goal_center());
        let angle_backward = me.Physics.loc().to_2d().angle_to(my_goal_center_2d());

        let angle_offense = (angle_ball - angle_forward).normalize_angle().abs();
        let angle_defense = (angle_ball - angle_backward).normalize_angle().abs();

        let theta = if angle_offense < angle_defense {
            ctx.eeg.log("hitting toward enemy goal");
            feasible_hit_angle_toward(ball, me.Physics.loc().to_2d(), goal, PI / 6.0)
        } else {
            ctx.eeg.log("hitting away from own goal");
            feasible_hit_angle_away(ball, me.Physics.loc().to_2d(), avoid, PI / 6.0)
        };
        let aim_loc = WallRayCalculator::calc_ray(ball, theta);
        Action::call(BounceShot::new().with_target_loc(aim_loc))
    }
}
