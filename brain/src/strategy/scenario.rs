use crate::{
    plan::ball::{BallFrame, BallPredictor, BallTrajectory},
    predict::intercept::NaiveIntercept,
    strategy::game::Game,
    utils::{Wall, WallRayCalculator},
};
use common::prelude::*;
use lazycell::LazyCell;
use ordered_float::NotNan;
use simulate::{linear_interpolate, Car1D};
use std::f32::{self, consts::PI};

pub struct Scenario<'a> {
    packet: &'a rlbot::ffi::LiveDataPacket,
    pub game: &'a Game<'a>,
    ball_predictor: &'a BallPredictor,
    ball_prediction: LazyCell<BallTrajectory>,
    me_intercept: LazyCell<Option<NaiveIntercept>>,
    enemy_intercept: LazyCell<Option<(&'a rlbot::ffi::PlayerInfo, NaiveIntercept)>>,
    possession: LazyCell<f32>,
    push_wall: LazyCell<Wall>,
    impending_concede: LazyCell<Option<BallFrame>>,
    enemy_shoot_score_seconds: LazyCell<f32>,
}

impl<'a> Scenario<'a> {
    pub const POSSESSION_CONTESTABLE: f32 = 0.5;
    pub const POSSESSION_SATURATED: f32 = 5.0;

    pub fn new(
        game: &'a Game,
        ball_predictor: &'a BallPredictor,
        packet: &'a rlbot::ffi::LiveDataPacket,
    ) -> Scenario<'a> {
        Scenario {
            packet,
            game,
            ball_predictor,
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
        self.ball_prediction
            .borrow_with(|| self.ball_predictor.predict(self.packet))
    }

    pub fn me_intercept(&self) -> Option<&NaiveIntercept> {
        if !self.me_intercept.filled() {
            self.race();
        }
        self.me_intercept.borrow().unwrap().as_ref()
    }

    pub fn enemy_intercept(&self) -> Option<&(&'a rlbot::ffi::PlayerInfo, NaiveIntercept)> {
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
        let blitz_me = simulate_ball_blitz(self.ball_prediction(), self.game.me());
        let blitz_enemy = self
            .game
            .cars(self.game.enemy_team)
            .map(|enemy| (enemy, simulate_ball_blitz(self.ball_prediction(), enemy)))
            .filter_map(|(enemy, intercept)| intercept.map(|i| (enemy, i)))
            .min_by_key(|(_enemy, intercept)| NotNan::new(intercept.time).unwrap());

        let possession = match (&blitz_me, &blitz_enemy) {
            (Some(me), Some((_, enemy))) => enemy.time - me.time,
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
                None => self.ball_prediction().last().loc,
            };
            let me_loc = self.game.me().Physics.loc();
            let point = WallRayCalculator::calculate(me_loc.to_2d(), intercept_loc.to_2d());
            WallRayCalculator::wall_for_point(self.game, point)
        })
    }

    /// If nobody touches the ball, will it end up in our goal?
    pub fn impending_concede(&self) -> Option<&BallFrame> {
        self.impending_concede
            .borrow_with(|| {
                self.ball_prediction()
                    .iter_step_by(0.5)
                    .find(|ball| self.game.own_goal().ball_is_scored(ball.loc))
            })
            .as_ref()
    }

    /// If the enemy can shoot, guesstimate the number of seconds before the
    /// shot would be scored.
    pub fn enemy_shoot_score_seconds(&self) -> f32 {
        *self.enemy_shoot_score_seconds.borrow_with(|| {
            let (car, intercept) = some_or_else!(self.enemy_intercept(), {
                return f32::INFINITY;
            });

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

fn blitz_start(car: &rlbot::ffi::PlayerInfo, ball_prediction: &BallTrajectory) -> Car1D {
    let ball_loc = ball_prediction.start().loc.to_2d();
    let car_vel = car.Physics.vel_2d();
    let car_to_ball = ball_loc - car.Physics.loc_2d();
    let speed_towards_ball = car_vel.dot(&car_to_ball.normalize());
    Car1D::new()
        .with_speed(speed_towards_ball.max(0.0))
        .with_boost(car.Boost as f32)
}

// Basically simulate a "race to the ball" (poorly) and guesstimate where our
// first possible intercept might be.
fn simulate_ball_blitz(
    ball_prediction: &BallTrajectory,
    car: &rlbot::ffi::PlayerInfo,
) -> Option<NaiveIntercept> {
    let mut sim = blitz_start(car, ball_prediction);
    let mut naive_result = None;

    for ball in ball_prediction.iter_step_by(0.125) {
        let dist_to_ball = (car.Physics.loc() - ball.loc).to_2d().norm();
        if sim.distance() >= dist_to_ball {
            naive_result = Some(ball);
            break;
        }
        sim.advance(ball.dt(), 1.0, true);
    }

    let naive_result = some_or_else!(naive_result, {
        return None;
    });
    let penalty = blitz_penalty(car, &naive_result);
    let ball = ball_prediction.at_time_or_last(naive_result.t + penalty);
    Some(NaiveIntercept {
        time: ball.t - ball_prediction.start().t,
        ball_loc: ball.loc,
        ball_vel: ball.vel,
        car_loc: ball.loc,
        car_speed: ball.vel.norm(),
        data: (),
    })
}

fn blitz_penalty(car: &rlbot::ffi::PlayerInfo, ball: &BallFrame) -> f32 {
    let ball_loc = ball.loc.to_2d();
    let ball_vel = ball.vel.to_2d();
    let car_vel = car.Physics.vel_2d();
    let car_forward = car.Physics.forward_axis_2d();
    let car_to_ball = ball_loc - car.Physics.loc_2d();

    let rotation_penalty = car_forward
        .rotation_to(&car_to_ball.to_axis())
        .angle()
        .abs()
        * 0.25;

    let speed_towards_ball = (car_vel - ball_vel).dot(&car_to_ball.normalize());
    let reverse_penalty = if speed_towards_ball < 0.0 {
        speed_towards_ball / -2000.0
    } else {
        0.0
    };

    rotation_penalty + reverse_penalty
}
