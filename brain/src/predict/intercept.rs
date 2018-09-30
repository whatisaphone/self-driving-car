use chip::Ball;
use nalgebra::Vector3;
use rlbot;
use simulate::{rl, Car1D};
use strategy::Context;
use utils::{ExtendPhysics, ExtendVector3};

const MAX_SIM_TIME: f32 = 5.0;

pub fn estimate_intercept_car_ball(
    ctx: &mut Context,
    car: &rlbot::PlayerInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Option<Intercept> {
    const DT: f32 = 1.0 / 60.0;

    // We don't want the center of the car to be at the center of the ball –
    // we want their meshes to barely be touching.
    const RADII: f32 = 240.0;

    let mut t = 0.0;
    let mut sim_car = Car1D::new(car.Physics.vel().norm()).with_boost(car.Boost);
    let mut sim_ball = Ball::new();
    sim_ball.set_pos(ctx.packet.GameBall.Physics.loc());
    sim_ball.set_vel(ctx.packet.GameBall.Physics.vel());
    sim_ball.set_omega(ctx.packet.GameBall.Physics.ang_vel());

    loop {
        t += DT;
        sim_ball.step(DT);
        sim_car.step(DT, 1.0, true);

        let target_traveled = (sim_ball.pos() - car.Physics.loc()).to_2d().norm() - RADII;
        if sim_car.distance_traveled() >= target_traveled {
            if predicate(t, &sim_ball.pos(), &sim_ball.vel()) {
                break;
            }
        }

        if t >= MAX_SIM_TIME {
            return None;
        }
    }

    let intercept_loc = sim_ball.pos() - (sim_ball.pos() - car.Physics.loc()).normalize() * RADII;
    let intercept = Intercept {
        time: t,
        ball_loc: sim_ball.pos(),
        ball_vel: sim_ball.vel(),
        car_loc: intercept_loc,
        car_speed: sim_car.speed(),
    };
    Some(intercept)
}

pub fn is_sane_ball_loc(loc: Vector3<f32>) -> bool {
    loc.x.abs() < rl::FIELD_MAX_X && loc.y.abs() < rl::FIELD_MAX_Y
}

pub struct Intercept {
    pub time: f32,
    pub ball_loc: Vector3<f32>,
    pub ball_vel: Vector3<f32>,
    pub car_loc: Vector3<f32>,
    pub car_speed: f32,
}
