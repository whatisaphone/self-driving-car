use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use eeg::{color, Drawable, EEG};
use mechanics::simple_steer_towards;
use nalgebra::Vector2;
use rlbot;
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

    fn capture(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Option<Action> {
        if Self::on_flat_ground(packet) {
            return Some(Action::Return);
        }
        None
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);

        // Should we reverse down the wall?
        if me.OnGround && me.Physics.rot().pitch() >= 30.0_f32.to_radians() {
            return Action::call(ReverseSlide::new());
        }

        Action::call(Simple::new())
    }
}

struct Simple;

impl Simple {
    pub fn new() -> Simple {
        Simple
    }
}

impl Behavior for Simple {
    fn name(&self) -> &'static str {
        "Simple"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);

        Action::Yield(rlbot::PlayerInput {
            Throttle: 1.0,
            Steer: simple_steer_towards(&me.Physics, Vector2::zeros()),
            ..Default::default()
        })
    }
}

struct ReverseSlide;

impl ReverseSlide {
    pub fn new() -> ReverseSlide {
        ReverseSlide
    }
}

impl Behavior for ReverseSlide {
    fn name(&self) -> &'static str {
        "ReverseSlide"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("Hilarity ensues", color::GREEN));
        Action::Yield(rlbot::PlayerInput {
            Throttle: -1.0,
            Steer: 1.0,
            Handbrake: true,
            ..Default::default()
        })
    }
}
