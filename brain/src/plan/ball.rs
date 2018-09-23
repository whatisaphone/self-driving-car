use nalgebra::Vector3;
use rlbot;
use simulate::chip::Ball;
use utils::ExtendPhysics;

const MAX_SIM_TIME: f32 = 3.0;

pub fn predict_ball(
    ball: &rlbot::BallInfo,
    predicate: impl Fn(f32, &Vector3<f32>, &Vector3<f32>) -> bool,
) -> Option<Ball> {
    const DT: f32 = 1.0 / 60.0;

    let mut t = 0.0;
    let mut sim_ball = Ball::new(
        ball.Physics.loc(),
        ball.Physics.vel(),
        ball.Physics.ang_vel(),
    );

    loop {
        t += DT;
        sim_ball.step(DT);

        if predicate(t, &sim_ball.loc(), &sim_ball.vel()) {
            return Some(sim_ball);
        }

        if t >= MAX_SIM_TIME {
            return None;
        }
    }
}
