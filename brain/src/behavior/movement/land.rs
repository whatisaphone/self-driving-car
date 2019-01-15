use crate::{
    eeg::{color, Drawable},
    routing::{models::CarState, plan::avoid_goal_wall_waypoint},
    strategy::{Action, Behavior, Context},
};
use common::{kinematics::kinematic_time, prelude::*, rl};
use derive_new::new;
use nalgebra::{Unit, Vector2, Vector3};
use nameof::name_of_type;
use std::f32::consts::PI;

#[derive(new)]
pub struct Land;

impl Behavior for Land {
    fn name(&self) -> &str {
        name_of_type!(Land)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let me = ctx.me();

        if me.OnGround {
            return Action::Return;
        }

        if me.Physics.ang_vel().norm() >= 5.25 {
            // This is a minor hack for statelessness. A car's max angular velocity is 5.5
            // rad/sec, so if we're near that limit, we're probably in the middle of a
            // dodge. Just let it happen.
            ctx.eeg
                .draw(Drawable::print("waiting until after flip", color::GREEN));
            Action::Yield(Default::default())
        } else {
            ctx.eeg.draw(Drawable::print("air rolling", color::GREEN));

            let facing = choose_facing(ctx);

            // Boost towards the ground if we're floating helplessly
            let (forward, boost);
            if me.Boost > 0 {
                let time_to_ground = kinematic_time(
                    -me.Physics.loc().z + rl::OCTANE_NEUTRAL_Z,
                    me.Physics.vel().z,
                    rl::GRAVITY,
                )
                .unwrap();
                let down = time_to_ground >= 0.75;
                forward = if down {
                    facing.rotation_to(&-Vector3::z_axis()).powf(0.8) * facing
                } else {
                    facing
                };

                let nose_down_angle = me.Physics.forward_axis().angle_to(&-Vector3::z_axis());
                boost = down && nose_down_angle < PI / 3.0;

                ctx.eeg.print_time("time_to_ground", time_to_ground);
                ctx.eeg.print_value("down", format!("{}", down));
                ctx.eeg.print_angle("nose_down_angle", nose_down_angle);
            } else {
                forward = facing;
                boost = false;

                ctx.eeg.draw(Drawable::print("no boost", color::GREEN));
            }

            let (pitch, yaw, roll) = dom::get_pitch_yaw_roll(me, forward, Vector3::z_axis());
            Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Pitch: pitch,
                Yaw: yaw,
                Roll: roll,
                Boost: boost,
                ..Default::default()
            })
        }
    }
}

impl Land {
    /// How far deep in enemy territory are we?
    pub(super) fn defensiveness(ctx: &mut Context<'_>) -> f32 {
        let safe = ctx.game.own_back_wall_center();
        let danger = ctx.game.enemy_back_wall_center();
        let me_loc = ctx.me().Physics.loc_2d();
        (me_loc - danger).norm() / (me_loc - safe).norm()
    }
}

fn choose_facing(ctx: &mut Context<'_>) -> Unit<Vector3<f32>> {
    let me = ctx.me();

    if me.Physics.loc().y.abs() >= ctx.game.field_max_y() {
        // If we're going to land in the goal, land in a convenient direction to
        // immediately drive out of the goal towards the ball.
        ctx.eeg
            .draw(Drawable::print("landing in a goal", color::GREEN));
        face_the_ball(ctx)
    } else if me.Physics.vel_2d().norm() < 800.0 {
        // If we're not moving much, we have no momentum to conserve, so face the ball.
        ctx.eeg.draw(Drawable::print("no momentum", color::GREEN));
        face_the_ball(ctx)
    } else if Land::defensiveness(ctx) >= 7.0 {
        // If we're playing defense, forget momentum, try to stay ready to challenge the
        // ball.
        ctx.eeg
            .draw(Drawable::print("defensive positioning", color::GREEN));
        face_the_ball(ctx)
    } else {
        // Conserve our momentum (i.e. don't skid on landing)
        ctx.eeg
            .draw(Drawable::print("conserving momentum", color::GREEN));
        me.Physics.vel_2d()
    }
    .to_3d(0.0)
    .to_axis()
}

fn face_the_ball(ctx: &mut Context<'_>) -> Vector2<f32> {
    let me = ctx.me();
    let mut start = CarState::from(me);
    start.loc += start.vel * 1.0;
    let ball_loc = ctx
        .scenario
        .ball_prediction()
        .at_time_or_last(0.5)
        .loc
        .to_2d();
    let target_loc = avoid_goal_wall_waypoint(&start, ball_loc).unwrap_or(ball_loc);
    target_loc - me.Physics.loc_2d()
}
