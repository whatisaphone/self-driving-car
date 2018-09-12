use behavior::{Action, Behavior};
use eeg::{Drawable, EEG};
use maneuvers::{drive_towards, GetToFlatGround};
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

        Action::Yield(drive_towards(packet, eeg, self.target_loc))
    }
}
