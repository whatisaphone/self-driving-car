use common::prelude::*;
use lazycell::LazyCell;
use nalgebra::Point3;
use plan::ball::{BallFrame, BallTrajectory};
use predict::intercept::NaiveIntercept;
use rlbot;
use simulate::{linear_interpolate, Car1D};
use std::f32::{self, consts::PI};
use strategy::game::Game;
use utils::{one_v_one, ExtendPoint3, ExtendVector3, Wall, WallRayCalculator};

pub struct Scenario<'a> {
    packet: &'a rlbot::ffi::LiveDataPacket,
    game: Game<'a>,
    ball_prediction: LazyCell<BallTrajectory>,
    me_intercept: LazyCell<Option<NaiveIntercept>>,
    enemy_intercept: LazyCell<Option<NaiveIntercept>>,
    possession: LazyCell<f32>,
    push_wall: LazyCell<Wall>,
    impending_concede: LazyCell<Option<&'a BallFrame>>,
    enemy_shoot_score_seconds: LazyCell<f32>,
}

impl<'a> Scenario<'a> {
    pub const POSSESSION_CONTESTABLE: f32 = 0.5;
    pub const POSSESSION_SATURATED: f32 = 5.0;

    pub fn new(packet: &'a rlbot::ffi::LiveDataPacket) -> Scenario<'a> {
        Scenario {
            packet,
            game: Game::new(packet),
            ball_prediction: LazyCell::new(),
            me_intercept: LazyCell::new(),
            enemy_intercept: LazyCell::new(),
            possession: LazyCell::new(),
            push_wall: LazyCell::new(),
            impending_concede: LazyCell::new(),
            enemy_shoot_score_seconds: LazyCell::new(),
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

    /// If nobody touches the ball, will it end up in our goal?
    #[allow(dead_code)]
    pub fn impending_concede(&'a self) -> Option<&'a BallFrame> {
        *self.impending_concede.borrow_with(|| {
            self.ball_prediction()
                .iter()
                .find(|ball| self.game.own_goal().ball_is_scored(ball.loc))
        })
    }

    /// If the enemy can shoot, guesstimate the number of seconds before the
    /// shot would be scored.
    #[allow(dead_code)]
    pub fn enemy_shoot_score_seconds(&self) -> f32 {
        *self.enemy_shoot_score_seconds.borrow_with(|| {
            let intercept = some_or_else!(self.enemy_intercept(), {
                return f32::INFINITY;
            });

            let car = self.game.enemy();
            let car_to_ball = intercept.ball_loc.to_2d() - car.Physics.loc_2d();
            let ball_to_goal = self.game.own_goal().center_2d - intercept.ball_loc.to_2d();

            let ball_scoring_speed = intercept.ball_vel.to_2d().dot(&ball_to_goal.normalize());
            let car_vel = car_to_ball.normalize() * intercept.car_speed;
            let impulse_guess = car_vel.dot(&ball_to_goal.normalize()).max(0.0) * 2.0;
            let angle_factor = linear_interpolate(
                &[PI / 6.0, PI / 2.0],
                &[1.0, 0.0],
                car_to_ball.rotation_to(ball_to_goal).angle().abs(),
            );

            let shot_speed = ball_scoring_speed + impulse_guess * angle_factor;
            if shot_speed < 1.0 {
                f32::INFINITY
            } else {
                ball_to_goal.norm() / shot_speed
            }
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
