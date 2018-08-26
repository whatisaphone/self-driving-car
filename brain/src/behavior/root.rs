use behavior::shoot::Shoot;
use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::FiftyFifty;
use rlbot;
use utils::{one_v_one, ExtendPhysics};

pub struct RootBehavior;

impl RootBehavior {
    pub fn new() -> RootBehavior {
        RootBehavior
    }
}

impl Behavior for RootBehavior {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let possession = possession(packet);

        info!("possession: {:?}", possession);

        match possession {
            Possession::Me => Action::call(Shoot::new()),
            Possession::Enemy => Action::call(Shoot::new()),
            Possession::Unsure => Action::call(Shoot::new()),
        }
    }
}

fn possession(packet: &rlbot::LiveDataPacket) -> Possession {
    let (me, enemy) = one_v_one(packet);
    let dist_me_ball = (me.Physics.loc() - packet.GameBall.Physics.loc()).norm();
    let dist_enemy_ball = (enemy.Physics.loc() - packet.GameBall.Physics.loc()).norm();
    match dist_me_ball / dist_enemy_ball {
        x if x < 0.75 => Possession::Me,
        x if x < 1.33 => Possession::Unsure,
        _ => Possession::Enemy,
    }
}

#[derive(Debug)]
enum Possession {
    Me,
    Enemy,
    Unsure,
}
