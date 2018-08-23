use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::misc::simple_steer_towards;
use nalgebra::Vector3;
use rlbot;
use simulate::Car1D;
use utils::{my_car, ExtendPhysics};

pub struct GroundAccelToLoc {
    target_loc: Vector3<f32>,
    target_time: f32,
}

impl GroundAccelToLoc {
    pub fn new(target_loc: Vector3<f32>, target_time: f32) -> GroundAccelToLoc {
        GroundAccelToLoc {
            target_loc,
            target_time,
        }
    }
}

impl Behavior for GroundAccelToLoc {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("GroundAccelToLoc", color::YELLOW));

        let me = my_car(packet);
        let distance = (me.Physics.loc() - self.target_loc).norm();
        let time_remaining = self.target_time - packet.GameInfo.TimeSeconds;

        eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));

        let mut result = rlbot::PlayerInput::default();
        result.Steer = simple_steer_towards(&me.Physics, self.target_loc);
        if !estimate_approach(&me.Physics, distance, time_remaining) {
            result.Throttle = 1.0;
        }

        Action::Yield(result)
    }
}

/// Starting at `origin`, if we go pedal to the metal for `time` seconds, will
/// we have traveled `distance`?
fn estimate_approach(origin: &rlbot::Physics, distance: f32, time: f32) -> bool {
    const DT: f32 = 1.0 / 60.0;

    let mut t = 2.0 / 60.0; // Start a few frames later to account for input lag.
    let mut sim_car = Car1D::new(origin.vel().norm());

    while t < time {
        t += DT;
        sim_car.step(DT, 1.0, false);

        if sim_car.distance_traveled() >= distance {
            return true;
        }
    }

    false
}
