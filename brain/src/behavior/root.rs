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

        let plan = eval(packet);
        eeg.log(format!("{:?}", plan));

        Some(Action::Call(plan.to_behavior()))
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        self.last_eval = Some(packet.GameInfo.TimeSeconds);

        let plan = eval(packet);
        eeg.log(format!("{:?}", plan));

        Action::Call(plan.to_behavior())
    }
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn eval(packet: &rlbot::LiveDataPacket) -> Plan {
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
    let situation = eval_situation(ball_at_interception.loc());
    println!("{:?}", situation);

    let possession = match me_time / enemy_time {
        x if x < 0.75 => Possession::Me,
        x if x < 1.33 => Possession::Unsure,
        _ => Possession::Enemy,
    };

    match (situation, possession) {
        (Situation::OwnBox, _) => Plan::Defense,
        (_, Possession::Me) => Plan::Offense,
        (_, Possession::Unsure) => Plan::Offense,
        (_, Possession::Enemy) => Plan::Defense,
    }
}

fn eval_situation(loc: Vector3<f32>) -> Situation {
    match () {
        () if loc.y > 2500.0 && loc.x.abs() < 1800.0 => Situation::EnemyBox,
        () if loc.y > 2500.0 => Situation::EnemyCorner,
        () if loc.y < -2500.0 && loc.x.abs() < 1800.0 => Situation::OwnBox,
        () if loc.y < -2500.0 => Situation::OwnCorner,
        () => Situation::Midfield,
    }
}

#[derive(Debug)]
enum Situation {
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
