use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use eeg::{color, Drawable, EEG};
use mechanics::{simple_steer_towards, QuickJumpAndDodge};
use nalgebra::Vector3;
use rlbot;
use simulate::rl;
use std::f32::consts::PI;
use utils::{my_car, ExtendPhysics};

pub struct GetToFlatGround;

impl GetToFlatGround {
    pub fn new() -> GetToFlatGround {
        GetToFlatGround
    }

    pub fn on_flat_ground(packet: &rlbot::LiveDataPacket) -> bool {
        let me = my_car(packet);
        me.OnGround
            && me.Physics.rot().pitch().abs() < 5.0_f32.to_radians()
            && me.Physics.rot().roll().abs() < 5.0_f32.to_radians()
    }
}

impl Behavior for GetToFlatGround {
    fn name(&self) -> &'static str {
        "GetToFlatGround"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if Self::on_flat_ground(packet) {
            return Action::Return;
        }

        let me = my_car(packet);

        return Action::Yield(rlbot::PlayerInput {
            Throttle: 1.0,
            Steer: simple_steer_towards(&me.Physics, Vector3::zeros()),
            ..Default::default()
        });
    }
}
