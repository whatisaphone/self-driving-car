use nalgebra::Vector3;
use plan::ball::BallTrajectory;
use rlbot;
use simulate::Car1D;
use utils::{one_v_one, ExtendPhysics, ExtendVector3, Wall, WallRayCalculator};

pub struct Scenario<'a> {
    packet: &'a rlbot::LiveDataPacket,
    ball_prediction: Option<BallTrajectory>,
    me_intercept: Option<Option<(f32, Vector3<f32>)>>,
    enemy_intercept: Option<Option<(f32, Vector3<f32>)>>,
    possession: Option<f32>,
    push_wall: Option<Option<Wall>>,
}

impl<'a> Scenario<'a> {
    pub const POSSESSION_CONTESTABLE: f32 = 0.5;
    pub const POSSESSION_SATURATED: f32 = 5.0;

    pub fn new(packet: &'a rlbot::LiveDataPacket) -> Scenario<'a> {
        Scenario {
            packet,
            ball_prediction: None,
            me_intercept: None,
            enemy_intercept: None,
            possession: None,
            push_wall: None,
        }
    }

    pub fn ball_prediction(&mut self) -> &BallTrajectory {
        if let Some(ref x) = self.ball_prediction {
            return x;
        }

        self.ball_prediction = Some(BallTrajectory::predict(&self.packet));
        self.ball_prediction.as_ref().unwrap()
    }

    pub fn me_intercept(&mut self) -> Option<(f32, Vector3<f32>)> {
        self.possession();
        self.me_intercept.unwrap()
    }

    pub fn enemy_intercept(&mut self) -> Option<(f32, Vector3<f32>)> {
        self.possession();
        self.enemy_intercept.unwrap()
    }

    /// Number of seconds I can reach the ball before the opponent
    pub fn possession(&mut self) -> f32 {
        self.possession.unwrap_or_else(|| {
            let (blitz_me, blitz_enemy) = simulate_ball_blitz(self.packet, self.ball_prediction());

            self.me_intercept = Some(blitz_me);
            self.enemy_intercept = Some(blitz_enemy);
            self.possession = Some(match (blitz_me, blitz_enemy) {
                (Some((me, _)), Some((en, _))) => me - en,
                (Some(_), None) => Self::POSSESSION_SATURATED,
                (None, Some(_)) => -Self::POSSESSION_SATURATED,
                (None, None) => {
                    // Very bizarre situation? Let's just go for the ball here.
                    Self::POSSESSION_SATURATED
                }
            });

            self.possession.unwrap()
        })
    }

    /// If I blitz to the ball and hit it straight-on, where will it go?
    pub fn push_wall(&mut self) -> Option<Wall> {
        self.push_wall.unwrap_or_else(|| {
            self.push_wall = Some(self.me_intercept().map(|(_t, loc)| {
                let (me, _enemy) = one_v_one(self.packet);
                eval_push_wall(&me.Physics.loc(), &loc)
            }));
            self.push_wall.unwrap()
        })
    }
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn simulate_ball_blitz(
    packet: &rlbot::LiveDataPacket,
    ball_prediction: &BallTrajectory,
) -> (Option<(f32, Vector3<f32>)>, Option<(f32, Vector3<f32>)>) {
    let (me, enemy) = one_v_one(packet);
    let t = 0.0;
    let mut sim_me = Car1D::new(me.Physics.vel().norm()).with_boost(me.Boost);
    let mut sim_enemy = Car1D::new(enemy.Physics.vel().norm()).with_boost(enemy.Boost);

    let mut me_time = None;
    let mut enemy_time = None;

    for ball in ball_prediction.iter() {
        if me_time.is_none() {
            sim_me.step(ball.dt(), 1.0, true);
            let me_dist_to_ball = (me.Physics.loc() - ball.loc).to_2d().norm();
            if sim_me.distance_traveled() >= me_dist_to_ball {
                me_time = Some((t, ball.loc));
            }
        }

        if enemy_time.is_none() {
            sim_enemy.step(ball.dt(), 1.0, true);
            let enemy_dist_to_ball = (enemy.Physics.loc() - ball.loc).to_2d().norm();
            if sim_enemy.distance_traveled() >= enemy_dist_to_ball {
                enemy_time = Some((t, ball.loc));
            }
        }

        if me_time.is_some() && enemy_time.is_some() {
            break;
        }
    }

    (me_time, enemy_time)
}

fn eval_push_wall(car: &Vector3<f32>, ball: &Vector3<f32>) -> Wall {
    let point = WallRayCalculator::calculate(car.to_2d(), ball.to_2d());
    WallRayCalculator::wall_for_point(point)
}
