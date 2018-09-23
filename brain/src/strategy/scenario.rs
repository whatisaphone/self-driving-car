use nalgebra::Vector3;
use rlbot;
use simulate::{chip::Ball, Car1D};
use utils::{one_v_one, ExtendPhysics, ExtendVector3, Wall, WallRayCalculator};

pub struct Scenario<'a> {
    packet: &'a rlbot::LiveDataPacket,
    me_intercept: Option<(f32, Vector3<f32>)>,
    enemy_intercept: Option<(f32, Vector3<f32>)>,
    possession: Option<f32>,
    push_wall: Option<Wall>,
}

impl<'a> Scenario<'a> {
    pub const POSSESSION_CONTESTABLE: f32 = 0.5;

    pub fn new(packet: &'a rlbot::LiveDataPacket) -> Scenario<'a> {
        Scenario {
            packet,
            me_intercept: None,
            enemy_intercept: None,
            possession: None,
            push_wall: None,
        }
    }

    pub fn me_intercept(&mut self) -> (f32, Vector3<f32>) {
        self.possession();
        self.me_intercept.unwrap()
    }

    pub fn enemy_intercept(&mut self) -> (f32, Vector3<f32>) {
        self.possession();
        self.enemy_intercept.unwrap()
    }

    /// Number of seconds I can reach the ball before the opponent
    pub fn possession(&mut self) -> f32 {
        self.possession.unwrap_or_else(|| {
            let (me, _enemy) = one_v_one(self.packet);

            let ((blitz_me_time, blitz_me_loc), (blitz_enemy_time, blitz_enemy_loc)) =
                simulate_ball_blitz(self.packet);

            self.me_intercept = Some((blitz_me_time, blitz_me_loc));
            self.enemy_intercept = Some((blitz_enemy_time, blitz_enemy_loc));
            self.possession = Some(blitz_me_time - blitz_enemy_time);
            self.push_wall = Some(eval_push_wall(&me.Physics.loc(), &blitz_me_loc));

            self.possession.unwrap()
        })
    }

    /// If I blitz to the ball and hit it straight-on, where will it go?
    pub fn push_wall(&mut self) -> Wall {
        self.possession();
        self.push_wall.unwrap()
    }
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn simulate_ball_blitz(
    packet: &rlbot::LiveDataPacket,
) -> ((f32, Vector3<f32>), (f32, Vector3<f32>)) {
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
                me_time = Some((t, sim_ball.loc()));
            }
        }

        if enemy_time.is_none() {
            sim_enemy.step(DT, 1.0, true);
            if sim_enemy.distance_traveled()
                >= (enemy.Physics.loc() - sim_ball.loc()).to_2d().norm()
            {
                enemy_time = Some((t, sim_ball.loc()));
            }
        }
    }

    (me_time.unwrap(), enemy_time.unwrap())
}

fn eval_push_wall(car: &Vector3<f32>, ball: &Vector3<f32>) -> Wall {
    let point = WallRayCalculator::calculate(car.to_2d(), ball.to_2d());
    WallRayCalculator::wall_for_point(point)
}
