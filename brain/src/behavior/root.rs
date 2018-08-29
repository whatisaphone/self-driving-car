use behavior::shoot::Shoot;
use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::FiftyFifty;
use rlbot;
use simulate::{Ball, Car1D};
use utils::{one_v_one, ExtendPhysics, ExtendVector3};

pub struct RootBehavior {
    last_eval: Option<f32>,
}

impl RootBehavior {
    pub fn new() -> RootBehavior {
        RootBehavior { last_eval: None }
    }
}

impl Behavior for RootBehavior {
    fn name(&self) -> &'static str {
        "RootBehavior"
    }

    fn capture(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Option<Action> {
        match self.last_eval {
            // If we have already checked recently, bail.
            Some(t) if packet.GameInfo.TimeSeconds < t + 1.0 => return None,
            // If we haven't, continue.
            _ => {}
        }

        let possession = possession(packet);

        info!("possession: {:?}", possession);

        self.last_eval = Some(packet.GameInfo.TimeSeconds);

        match possession {
            Possession::Me => Some(Action::call(Shoot::new())),
            Possession::Enemy => None, // TODO
            Possession::Unsure => None,
        }
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let possession = possession(packet);

        info!("possession: {:?}", possession);

        self.last_eval = Some(packet.GameInfo.TimeSeconds);

        match possession {
            Possession::Me => Action::call(Shoot::new()),
            Possession::Enemy => Action::call(Shoot::new()), // TODO
            Possession::Unsure => Action::call(FiftyFifty::new()),
        }
    }
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn possession(packet: &rlbot::LiveDataPacket) -> Possession {
    const DT: f32 = 1.0 / 60.0;

    let (me, enemy) = one_v_one(packet);
    let mut t = 0.0;
    let mut sim_ball = Ball::new(
        packet.GameBall.Physics.loc(),
        packet.GameBall.Physics.vel(),
        packet.GameBall.Physics.ang_vel(),
    );
    let mut sim_me = Car1D::new(me.Physics.vel().norm()).with_boost(me.Boost);
    let mut sim_enemy = Car1D::new(enemy.Physics.vel().norm()).with_boost(enemy.Boost);

    let mut me_time = None;
    let mut enemy_time = None;

    while me_time.is_none() || enemy_time.is_none() {
        t += DT;
        sim_ball.step(DT);

        if me_time.is_none() {
            sim_me.step(DT, 1.0, true);
            if sim_me.distance_traveled() >= (me.Physics.loc() - sim_ball.loc()).to_2d().norm() {
                me_time = Some(t);
            }
        }

        if enemy_time.is_none() {
            sim_enemy.step(DT, 1.0, true);
            if sim_enemy.distance_traveled()
                >= (enemy.Physics.loc() - sim_ball.loc()).to_2d().norm()
            {
                enemy_time = Some(t);
            }
        }
    }

    match me_time.unwrap() / enemy_time.unwrap() {
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
