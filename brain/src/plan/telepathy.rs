use crate::{plan::hit_angle::feasible_hit_angle_toward, strategy::Context};
use common::prelude::*;
use nalgebra::{Unit, Vector2};
use std::f32::consts::PI;

pub fn predict_enemy_hit_direction(ctx: &mut Context<'_>) -> Option<Unit<Vector2<f32>>> {
    let (enemy, intercept) = ctx.scenario.enemy_intercept()?;
    let enemy_loc = enemy.Physics.loc_2d();
    let likely_aim = feasible_hit_angle_toward(
        intercept.ball_loc.to_2d(),
        enemy_loc,
        ctx.game.own_goal().center_2d,
        PI / 4.0,
    );
    Some((likely_aim - enemy_loc).to_axis())
}
