use crate::{
    behavior::{
        higher_order::Chain,
        movement::{dodge::Dodge, drive_towards::drive_towards, yielder::Yielder},
    },
    eeg::Drawable,
    routing::{models::CarState, plan::avoid_goal_wall_waypoint},
    strategy::{Action, Behavior, Context, Priority},
};
use common::prelude::*;
use nalgebra::{Unit, Vector2, Vector3};
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;

pub struct GetToFlatGround;

impl GetToFlatGround {
    pub fn new() -> GetToFlatGround {
        GetToFlatGround
    }

    pub fn on_flat_ground(car: &rlbot::ffi::PlayerInfo) -> bool {
        car.OnGround
            && car.Physics.rot().pitch().abs() < 15.0_f32.to_radians()
            && car.Physics.rot().roll().abs() < 15.0_f32.to_radians()
    }
}

impl Behavior for GetToFlatGround {
    fn name(&self) -> &str {
        name_of_type!(GetToFlatGround)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        if Self::on_flat_ground(ctx.me()) {
            return Action::Return;
        }

        let me = ctx.me();

        if me.OnGround && me.Physics.roof_axis().angle(&-Vector3::z_axis()) < PI / 10.0 {
            // We're probably upside down under the ceiling of a goal
            Action::Yield(rlbot::ffi::PlayerInput {
                Jump: true,
                ..Default::default()
            })
        } else if me.OnGround && me.Physics.forward_axis().angle(&Vector3::z_axis()) < PI / 4.0 {
            // Our nose is pointed towards the sky. It's quicker to jump down than to drive.
            if me.Physics.vel().z >= 0.0 || me.Physics.loc().z >= 1000.0 {
                // Phase one of the reverse dismount: back up so we don't jump into the sky
                Action::Yield(rlbot::ffi::PlayerInput {
                    Throttle: -1.0,
                    ..Default::default()
                })
            } else {
                // Phase two of the reverse dismount: jump. The rotator below will make us land
                // on our wheels.
                let mut inputs = Vec::<Box<Behavior>>::with_capacity(3);
                inputs.push(Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Pitch: 1.0,
                        Jump: true,
                        ..Default::default()
                    },
                    0.1,
                )));

                if defensiveness(ctx) < 0.5 {
                    // We're probably way out of the game. Dodge towards our goal to get back to
                    // defense quicker.
                    let angle = (me.Physics.forward_axis().unwrap()
                        + me.Physics.roof_axis().unwrap())
                    .to_2d()
                    .rotation_to(ctx.game.own_back_wall_center() - ctx.me().Physics.loc_2d());

                    // Let go of jump
                    inputs.push(Box::new(Yielder::new(
                        rlbot::ffi::PlayerInput {
                            Pitch: 1.0,
                            Jump: false,
                            ..Default::default()
                        },
                        0.25,
                    )));
                    // Then dodge.
                    inputs.push(Box::new(Dodge::new().angle(angle)));
                }

                Action::call(Chain::new(Priority::Idle, inputs))
            }
        } else if me.OnGround {
            let target_loc =
                (me.Physics.loc() + me.Physics.rot() * Vector3::new(500.0, 0.0, 250.0)).to_2d();
            ctx.eeg
                .draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));
            Action::Yield(drive_towards(ctx, target_loc))
        } else if me.Physics.ang_vel().norm() >= 5.0 {
            // This is a minor hack for statelessness. We're probably in the middle of a
            // dodge. Just sit tight.
            Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                ..Default::default()
            })
        } else {
            let facing = choose_facing(ctx);

            // Boost towards the ground if we're floating helplessly
            let (forward, boost);
            if ctx.me().Boost > 0 {
                let down_amount = linear_interpolate(
                    &[500.0, 1000.0],
                    &[0.0, 1.0],
                    me.Physics.loc().z + me.Physics.vel().z.max(0.0) * 1.0,
                );
                forward = facing.rotation_to(&-Vector3::z_axis()).powf(down_amount) * facing;

                let nose_down_angle = me
                    .Physics
                    .forward_axis()
                    .rotation_to(&-Vector3::z_axis())
                    .angle();
                boost = down_amount > 0.0 && nose_down_angle < PI / 3.0;
            } else {
                forward = facing;
                boost = false;
            }

            let (pitch, yaw, roll) = dom::get_pitch_yaw_roll(ctx.me(), forward, Vector3::z_axis());
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

#[allow(clippy::if_same_then_else)]
fn choose_facing(ctx: &mut Context) -> Unit<Vector3<f32>> {
    let me = ctx.me();

    if me.Physics.loc().y.abs() >= ctx.game.field_max_y() {
        // If we're going to land in the goal, land in a convenient direction to
        // immediately drive out of the goal towards the ball.
        face_the_ball(ctx)
    } else if me.Physics.vel_2d().norm() < 500.0 {
        // If we're not moving much, we have no momentum to conserve, so face the ball.
        face_the_ball(ctx)
    } else if defensiveness(ctx) >= 7.0 {
        // If we're playing defense, forget momentum, try to stay ready to challenge the
        // ball.
        face_the_ball(ctx)
    } else {
        // Conserve our momentum (i.e. don't skid on landing)
        me.Physics.vel_2d()
    }
    .to_3d(0.0)
    .to_axis()
}

/// How far deep in enemy territory are we?
fn defensiveness(ctx: &mut Context) -> f32 {
    let safe = ctx.game.own_back_wall_center();
    let danger = ctx.game.enemy_back_wall_center();
    let me_loc = ctx.me().Physics.loc_2d();
    (me_loc - danger).norm() / (me_loc - safe).norm()
}

fn face_the_ball(ctx: &mut Context) -> Vector2<f32> {
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
