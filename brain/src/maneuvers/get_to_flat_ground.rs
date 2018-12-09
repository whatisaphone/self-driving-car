use crate::{
    behavior::{Action, Behavior},
    eeg::Drawable,
    maneuvers::drive_towards,
    routing::{models::CarState, plan::avoid_goal_wall_waypoint},
    strategy::Context,
};
use common::prelude::*;
use nalgebra::Vector3;
use rlbot;
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
        stringify!(GetToFlatGround)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
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
        } else if me.OnGround {
            let target_loc =
                (me.Physics.locp() + me.Physics.rot() * Vector3::new(500.0, 0.0, 250.0)).to_2d();
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
            let forward = if me.Physics.locp().y.abs() >= ctx.game.field_max_y() {
                // If we're going to land in the goal, land in a convenient direction to
                // immediately drive out of the goal towards the ball.
                let mut start = CarState::from(me);
                start.loc += start.vel * 1.0;
                let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
                let target_loc = avoid_goal_wall_waypoint(&start, ball_loc).unwrap_or(ball_loc);
                (target_loc - me.Physics.loc_2d())
            } else {
                me.Physics.vel().to_2d()
            }
            .to_3d(0.0)
            .to_axis();

            let (pitch, yaw, roll) = dom::get_pitch_yaw_roll(ctx.me(), forward, Vector3::z_axis());
            Action::Yield(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Pitch: pitch,
                Yaw: yaw,
                Roll: roll,
                ..Default::default()
            })
        }
    }
}
