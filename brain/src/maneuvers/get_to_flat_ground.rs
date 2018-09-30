use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use eeg::Drawable;
use maneuvers::drive_towards;
use nalgebra::Vector3;
use rlbot;
use std::f32::consts::PI;
use strategy::Context;
use utils::{my_car, ExtendPhysics, ExtendVector3};

pub struct GetToFlatGround;

impl GetToFlatGround {
    pub fn new() -> GetToFlatGround {
        GetToFlatGround
    }

    pub fn on_flat_ground(packet: &rlbot::LiveDataPacket) -> bool {
        let me = my_car(packet);
        me.OnGround
            && me.Physics.rot().pitch().abs() < 15.0_f32.to_radians()
            && me.Physics.rot().roll().abs() < 15.0_f32.to_radians()
    }
}

impl Behavior for GetToFlatGround {
    fn name(&self) -> &'static str {
        stringify!(GetToFlatGround)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if Self::on_flat_ground(ctx.packet) {
            return Action::Return;
        }

        let me = ctx.me();

        if me.OnGround
            && me.Physics.rot().pitch() < PI / 6.0
            && me.Physics.rot().roll().abs() >= PI / 2.0
        {
            // We're probably upside down under the ceiling of a goal
            Action::Yield(rlbot::PlayerInput {
                Jump: true,
                ..Default::default()
            })
        } else if me.OnGround {
            let target_loc =
                (me.Physics.loc() + me.Physics.rot() * Vector3::new(-500.0, 0.0, 0.0)).to_2d();
            ctx.eeg
                .draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));
            Action::Yield(drive_towards(ctx.packet, ctx.eeg, target_loc))
        } else if me.Physics.ang_vel().norm() >= 5.0 {
            // This is a minor hack for statelessness. We're probably in the middle of a
            // dodge. Just sit tight.
            Action::Yield(rlbot::PlayerInput {
                Throttle: 1.0,
                ..Default::default()
            })
        } else {
            Action::Yield(rlbot::PlayerInput {
                Throttle: 1.0,
                Pitch: (-me.Physics.rot().pitch()).max(-1.0).min(1.0),
                Roll: (-me.Physics.rot().roll()).max(-1.0).min(1.0),
                ..Default::default()
            })
        }
    }
}
