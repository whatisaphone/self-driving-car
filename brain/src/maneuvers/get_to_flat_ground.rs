use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use maneuvers::drive_towards;
use nalgebra::Vector2;
use rlbot;
use strategy::Context;
use utils::{my_car, ExtendPhysics};

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

        Action::Yield(drive_towards(ctx.packet, ctx.eeg, Vector2::zeros()))
    }
}
