use behavior::{Action, Behavior};
use eeg::{Drawable, EEG};
use maneuvers::{drive_towards, GetToFlatGround};
use nalgebra::Vector2;
use rlbot;
use utils::{my_car, ExtendPhysics};

const DURATION: f32 = 0.25;

pub struct HesitantDriveToLoc {
    start_time: f32,
    target_loc: Vector2<f32>,
}

impl HesitantDriveToLoc {
    pub fn begin(packet: &rlbot::LiveDataPacket, target_loc: Vector2<f32>) -> HesitantDriveToLoc {
        HesitantDriveToLoc {
            start_time: packet.GameInfo.TimeSeconds,
            target_loc,
        }
    }
}

impl Behavior for HesitantDriveToLoc {
    fn name(&self) -> &'static str {
        stringify!(HesitantDriveToLoc)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        // As part of being hesitant, give up frequently so the current strategy gets
        // re-evaluated.
        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;
        if elapsed >= DURATION {
            return Action::Return;
        }

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
