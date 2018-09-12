use behavior::{Action, Behavior};
use eeg::{Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::simple_steer_towards;
use nalgebra::Vector2;
use rlbot;
use utils::{my_car, ExtendPhysics};

pub struct HesitantDriveToLoc {
    target_loc: Vector2<f32>,
}

impl HesitantDriveToLoc {
    pub fn new(target_loc: Vector2<f32>) -> HesitantDriveToLoc {
        HesitantDriveToLoc { target_loc }
    }
}

impl Behavior for HesitantDriveToLoc {
    fn name(&self) -> &'static str {
        "HesitantDriveToLoc"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);

        eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
            me.Physics.rot(),
        ));

        // This behavior only operates in 2D
        if !GetToFlatGround::on_flat_ground(packet) {
            return Action::call(GetToFlatGround::new());
        }

        let mut result = rlbot::PlayerInput::default();
        result.Steer = simple_steer_towards(&me.Physics, self.target_loc);
        result.Throttle = 1.0;

        Action::Yield(result)
    }
}
