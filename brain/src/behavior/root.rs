use behavior::defense::Defense;
use behavior::offense::Offense;
use behavior::{Action, Behavior};
use eeg::EEG;
use nalgebra::Vector3;
use rlbot;
use simulate::{chip::Ball, Car1D};
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

        self.last_eval = Some(packet.GameInfo.TimeSeconds);

        let plan = eval(packet, eeg);
        eeg.log(format!("{:?}", plan));

        Some(Action::Call(plan.to_behavior()))
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        self.last_eval = Some(packet.GameInfo.TimeSeconds);

        let plan = eval(packet, eeg);
        eeg.log(format!("{:?}", plan));

        Action::Call(plan.to_behavior())
    }
}

fn eval(packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Plan {
    let situation = eval_situation(packet);
    let (place, possession) = eval_possession(packet);

    eeg.log(format!("{:?}", situation));
    eeg.log(format!("{:?}", place));
    eeg.log(format!("{:?}", possession));

    match (situation, place, possession) {
        (Situation::Retreat, _, _) => Plan::Defense,
        (_, Place::OwnBox, _) => Plan::Defense,
        (_, Place::OwnCorner, _) => Plan::Defense,
        (_, _, Possession::Me) => Plan::Offense,
        (_, _, Possession::Unsure) => Plan::Offense,
        (_, _, Possession::Enemy) => Plan::Defense,
    }
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn eval_possession(packet: &rlbot::LiveDataPacket) -> (Place, Possession) {
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
    let mut ball_at_interception = None;

    while me_time.is_none() || enemy_time.is_none() {
        t += DT;
        sim_ball.step(DT);

        if me_time.is_none() {
            sim_me.step(DT, 1.0, true);
            if sim_me.distance_traveled() >= (me.Physics.loc() - sim_ball.loc()).to_2d().norm() {
                me_time = Some(t);
                if ball_at_interception.is_none() {
                    ball_at_interception = Some(sim_ball.clone());
                }
            }
        }

        if enemy_time.is_none() {
            sim_enemy.step(DT, 1.0, true);
            if sim_enemy.distance_traveled()
                >= (enemy.Physics.loc() - sim_ball.loc()).to_2d().norm()
            {
                enemy_time = Some(t);
                if ball_at_interception.is_none() {
                    ball_at_interception = Some(sim_ball.clone());
                }
            }
        }
    }

    let mut me_time = me_time.unwrap();
    let mut enemy_time = enemy_time.unwrap();
    let mut ball_at_interception = ball_at_interception.unwrap();

    ball_at_interception.step(1.0); // Fast forward a bit
    let place = eval_ball(ball_at_interception.loc());

    let possession = match me_time / enemy_time {
        x if x < 0.75 => Possession::Me,
        x if x < 1.33 => Possession::Unsure,
        _ => Possession::Enemy,
    };

    (place, possession)
}

fn eval_ball(loc: Vector3<f32>) -> Place {
    match () {
        _ if loc.y > 2500.0 && loc.x.abs() < 1800.0 => Place::EnemyBox,
        _ if loc.y > 2500.0 => Place::EnemyCorner,
        _ if loc.y < -2500.0 && loc.x.abs() < 1800.0 => Place::OwnBox,
        _ if loc.y < -2500.0 => Place::OwnCorner,
        _ => Place::Midfield,
    }
}

fn eval_situation(packet: &rlbot::LiveDataPacket) -> Situation {
    let ball = packet.GameBall;
    let (me, enemy) = one_v_one(packet);

    if ball.Physics.vel().y < -500.0 && me.Physics.loc().y > ball.Physics.vel().y {
        Situation::Retreat
    } else {
        Situation::Unsure
    }
}

#[derive(Debug)]
enum Place {
    OwnBox,
    OwnCorner,
    Midfield,
    EnemyCorner,
    EnemyBox,
}

#[derive(Debug)]
enum Possession {
    Me,
    Enemy,
    Unsure,
}

#[derive(Debug)]
enum Situation {
    Retreat,
    Unsure,
}

#[derive(Debug)]
enum Plan {
    Offense,
    Defense,
}

impl Plan {
    fn to_behavior(&self) -> Box<Behavior> {
        match self {
            Plan::Offense => Box::new(Offense::new()),
            Plan::Defense => Box::new(Defense::new()),
        }
    }
}
