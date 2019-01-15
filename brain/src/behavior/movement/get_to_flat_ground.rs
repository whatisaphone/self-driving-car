use crate::{
    behavior::{
        higher_order::Chain,
        movement::{dodge::Dodge, drive_towards::drive_towards, land::Land, yielder::Yielder},
    },
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};
use common::prelude::*;
use nalgebra::Vector3;
use nameof::name_of_type;
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

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if Self::on_flat_ground(ctx.me()) {
            return Action::Return;
        }

        let me = ctx.me();

        if !me.OnGround {
            return Action::tail_call(Land::new());
        }

        if me.Physics.roof_axis().angle(&-Vector3::z_axis()) < PI / 10.0 {
            // We're probably upside down under the ceiling of a goal
            ctx.eeg.log(self.name(), "jumping while upside-down");
            Action::tail_call(Yielder::new(
                rlbot::ffi::PlayerInput {
                    Jump: true,
                    ..Default::default()
                },
                0.1,
            ))
        } else if me.Physics.forward_axis().angle(&Vector3::z_axis()) < PI / 4.0 {
            // Our nose is pointed towards the sky. It's quicker to jump down than to drive.
            ctx.eeg
                .draw(Drawable::print("nose pointed upwards", color::GREEN));
            if me.Physics.vel().z >= 0.0 || me.Physics.loc().z >= 1000.0 {
                // Phase one of the reverse dismount: back up so we don't jump into the sky
                ctx.eeg.draw(Drawable::print("backing up", color::GREEN));
                Action::Yield(rlbot::ffi::PlayerInput {
                    Throttle: -1.0,
                    ..Default::default()
                })
            } else {
                // Phase two of the reverse dismount: jump. The rotator below will make us land
                // on our wheels.
                ctx.eeg.log(self.name(), "jumping off the wall");
                let mut inputs = Vec::<Box<dyn Behavior>>::with_capacity(3);

                inputs.push(Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Pitch: 1.0,
                        Jump: true,
                        ..Default::default()
                    },
                    0.2,
                )));
                inputs.push(Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Pitch: 1.0,
                        Jump: false,
                        ..Default::default()
                    },
                    0.1,
                )));

                if Land::defensiveness(ctx) < 0.5 {
                    // We're probably way out of the game. Dodge towards our goal to get back to
                    // defense quicker.
                    ctx.eeg
                        .log(self.name(), "... and dodging to get back quick");

                    let angle = (me.Physics.forward_axis().unwrap()
                        + me.Physics.roof_axis().unwrap())
                    .to_2d()
                    .rotation_to(&(ctx.game.own_back_wall_center() - ctx.me().Physics.loc_2d()));

                    inputs.push(Box::new(Dodge::new().angle(angle)));
                }

                Action::tail_call(Chain::new(Priority::Idle, inputs))
            }
        } else {
            ctx.eeg
                .draw(Drawable::print("driving down the wall", color::GREEN));
            let target_loc =
                (me.Physics.loc() + me.Physics.rot() * Vector3::new(500.0, 0.0, 250.0)).to_2d();
            ctx.eeg
                .draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));
            Action::Yield(drive_towards(ctx, target_loc))
        }
    }
}
