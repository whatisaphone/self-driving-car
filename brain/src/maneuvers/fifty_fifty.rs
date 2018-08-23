use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::GroundAccelToLoc;
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use utils::{one_v_one, ExtendPhysics};

pub struct FiftyFifty;

impl Behavior for FiftyFifty {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print("FiftyFifty", color::YELLOW));

        let (me, _enemy) = one_v_one(packet);
        let (intercept_time, intercept_loc) = estimate_intercept_car_ball(&me, &packet.GameBall);

        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept_time),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostCar(intercept_loc, me.Physics.rot()));

        // TODO: this is not how this works…
        let mut child =
            GroundAccelToLoc::new(intercept_loc, packet.GameInfo.TimeSeconds + intercept_time);
        child.execute(packet, eeg)
    }
}
