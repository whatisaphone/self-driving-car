use behavior::{Behavior, Defense, Offense};
use eeg::EEG;
use maneuvers::FiftyFifty;
use nalgebra::Vector3;
use rlbot;
use simulate::chip::Ball;
use simulate::rl;
use simulate::Car1D;
use std::f32::consts::PI;
use std::time::Duration;
use strategy::Context;
use utils::{one_v_one, ExtendF32, ExtendPhysics, ExtendVector3, WALL_RAY_CALCULATOR};

pub fn baseline(ctx: &mut Context) -> Box<Behavior> {
    let (_possession, wall) = eval_possession(ctx);

    match wall {
        Wall::OwnGoal | Wall::OwnBackWall => Box::new(Defense::new()),
        _ => Box::new(Offense::new()),
    }
}

pub fn override_(ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>> {
    None
}

fn eval_possession(ctx: &mut Context) -> (f32, Wall) {
    let (me, enemy) = ctx.one_v_one();

    let (blitz_me_time, blitz_enemy_time, blitz_ball_loc) = simulate_ball_blitz(ctx.packet);
    let possession = blitz_me_time - blitz_enemy_time;
    ctx.eeg.log(format!("possession: {:.2}", possession));

    let push_wall = eval_push_wall(&me.Physics.loc(), &blitz_ball_loc, ctx.eeg);
    ctx.eeg.log(format!("push_wall: {:?}", push_wall));

    (possession, push_wall)
}

// This is a pretty naive and heavyweight implementation. Basically simulate a
// "race to the ball" and see if one player gets there much earlier than the
// other.
fn simulate_ball_blitz(packet: &rlbot::LiveDataPacket) -> (f32, f32, Vector3<f32>) {
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

    (
        me_time.unwrap(),
        enemy_time.unwrap(),
        ball_at_interception.unwrap().loc(),
    )
}

fn eval_push_wall(car: &Vector3<f32>, ball: &Vector3<f32>, eeg: &mut EEG) -> Wall {
    let point = WALL_RAY_CALCULATOR.calculate(car.to_2d(), ball.to_2d());
    let theta = f32::atan2(point.y, point.x);

    // For ease of math, center 0° on the enemy goal, 180° on own goal.
    let theta = (theta - PI / 2.0).normalize_angle().abs();

    eeg.log(format!("push_wall_theta: {:.0}°", theta.to_degrees()));

    // These are atan2(x, y) instead of atan2(y, x) because we rotated by 90° above.
    match theta {
        a if a < f32::atan2(rl::GOALPOST_X, rl::FIELD_MAX_Y) => Wall::EnemyGoal,
        a if a < f32::atan2(rl::FIELD_MAX_X, rl::FIELD_MAX_Y) => Wall::EnemyBackWall,
        a if a < f32::atan2(rl::FIELD_MAX_X, -rl::FIELD_MAX_Y) => Wall::Midfield,
        a if a < f32::atan2(rl::GOALPOST_X, -rl::FIELD_MAX_Y) => Wall::OwnBackWall,
        _ => Wall::OwnGoal,
    }
}

#[derive(Debug)]
enum Wall {
    EnemyGoal,
    EnemyBackWall,
    Midfield,
    OwnBackWall,
    OwnGoal,
}
