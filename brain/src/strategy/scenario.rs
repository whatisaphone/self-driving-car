use common::prelude::*;
use lazycell::LazyCell;
use nalgebra::Point3;
use plan::ball::BallTrajectory;
use predict::intercept::NaiveIntercept;
use rlbot;
use simulate::Car1D;
use utils::{one_v_one, ExtendPoint3, ExtendVector3, Wall, WallRayCalculator};

pub struct Scenario<'a> {
    packet: &'a rlbot::ffi::LiveDataPacket,
    ball_prediction: LazyCell<BallTrajectory>,
    me_intercept: LazyCell<Option<NaiveIntercept>>,
    enemy_intercept: LazyCell<Option<NaiveIntercept>>,
    possession: LazyCell<f32>,
    push_wall: LazyCell<Wall>,
}

impl<'a> Scenario<'a> {
    pub const POSSESSION_CONTESTABLE: f32 = 0.5;
    pub const POSSESSION_SATURATED: f32 = 5.0;

    pub fn new(packet: &'a rlbot::ffi::LiveDataPacket) -> Scenario<'a> {
        Scenario {
            packet,
            ball_prediction: LazyCell::new(),
            me_intercept: LazyCell::new(),
            enemy_intercept: LazyCell::new(),
            possession: LazyCell::new(),
            push_wall: LazyCell::new(),
        }
    }

    pub fn ball_prediction(&self) -> &BallTrajectory {
        let ball = self.packet.GameBall.Physics;
        self.ball_prediction
            .borrow_with(|| BallTrajectory::predict(ball.locp(), ball.vel(), ball.ang_vel()))
    }

    pub fn me_intercept(&self) -> Option<&NaiveIntercept> {
        if !self.me_intercept.filled() {
            self.race();
        }
        self.me_intercept.borrow().unwrap().as_ref()
    }

    pub fn enemy_intercept(&self) -> Option<&NaiveIntercept> {
        if !self.me_intercept.filled() {
            self.race();
        }
        self.enemy_intercept.borrow().unwrap().as_ref()
    }

    /// Number of seconds I can reach the ball before the opponent
    pub fn possession(&self) -> f32 {
        if !self.me_intercept.filled() {
            self.race();
        }
        *self.possession.borrow().unwrap()
    }

    fn race(&self) {
        let (blitz_me, blitz_enemy) = simulate_ball_blitz(self.packet, self.ball_prediction());
        let possession = match (&blitz_me, &blitz_enemy) {
            (Some(me), Some(enemy)) => enemy.time - me.time,
            _ => {
                // To avoid mexican standoffs, just pretend we have full possession so we go
                // for the ball.
                Self::POSSESSION_SATURATED
            }
        };

        self.me_intercept.fill(blitz_me).ok().unwrap();
        self.enemy_intercept.fill(blitz_enemy).ok().unwrap();
        self.possession.fill(possession).ok().unwrap();
    }

    /// If I blitz to the ball and hit it straight-on, where will it go?
    pub fn push_wall(&self) -> Wall {
        *self.push_wall.borrow_with(|| {
            let intercept_loc = match self.me_intercept() {
                Some(intercept) => intercept.ball_loc,
                None => self.ball_prediction().iter().last().unwrap().loc,
            };
            let (me, _enemy) = one_v_one(self.packet);
            eval_push_wall(&me.Physics.locp(), &intercept_loc)
        })
    }
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn simulate_ball_blitz(
    packet: &rlbot::ffi::LiveDataPacket,
    ball_prediction: &BallTrajectory,
) -> (Option<NaiveIntercept>, Option<NaiveIntercept>) {
    let (me, enemy) = one_v_one(packet);
    let mut t = 0.0;
    let mut sim_me = Car1D::new(me.Physics.vel().norm()).with_boost(me.Boost as f32);
    let mut sim_enemy = Car1D::new(enemy.Physics.vel().norm()).with_boost(enemy.Boost as f32);

    let mut me_result = None;
    let mut enemy_result = None;

    for ball in ball_prediction.iter() {
        t += ball.dt();

        if me_result.is_none() {
            sim_me.step(ball.dt(), 1.0, true);
            let me_dist_to_ball = (me.Physics.locp() - ball.loc).to_2d().norm();
            if sim_me.distance_traveled() >= me_dist_to_ball {
                me_result = Some(NaiveIntercept {
                    time: t,
                    ball_loc: ball.loc,
                    ball_vel: ball.vel,
                    car_loc: ball.loc,
                    car_speed: ball.vel.norm(),
                });
            }
        }

        if enemy_result.is_none() {
            sim_enemy.step(ball.dt(), 1.0, true);
            let enemy_dist_to_ball = (enemy.Physics.locp() - ball.loc).to_2d().norm();
            if sim_enemy.distance_traveled() >= enemy_dist_to_ball {
                enemy_result = Some(NaiveIntercept {
                    time: t,
                    ball_loc: ball.loc,
                    ball_vel: ball.vel,
                    car_loc: ball.loc,
                    car_speed: ball.vel.norm(),
                });
            }
        }

        if me_result.is_some() && enemy_result.is_some() {
            break;
        }
    }

    (me_result, enemy_result)
}

fn eval_push_wall(car: &Point3<f32>, ball: &Point3<f32>) -> Wall {
    let point = WallRayCalculator::calculate(car.to_2d(), ball.to_2d());
    WallRayCalculator::wall_for_point(point)
}
