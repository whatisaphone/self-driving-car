//! This is just a dumping ground of marginally-useful visualization stuff.

use common::ext::ExtendPhysics;
use rlbot;

pub fn draw_ball_prediction(rlbot: &rlbot::RLBot, packet: &rlbot::ffi::LiveDataPacket) {
    use chip::Ball;
    let mut ball = Ball::new();
    ball.set_pos(packet.GameBall.Physics.locp());
    ball.set_vel(packet.GameBall.Physics.vel());
    ball.set_omega(packet.GameBall.Physics.ang_vel());
    let mut prev = ball.pos();
    let mut rg = rlbot.begin_render_group(0);
    let green = rg.color_rgb(0, 255, 0);
    for _ in 0..(5 * 120 / 4) {
        for _ in 0..4 {
            ball.step(1.0 / 120.0);
        }
        let cur = ball.pos();
        rg.draw_line_3d(
            (prev.coords.x, prev.coords.y, prev.coords.z),
            (cur.coords.x, cur.coords.y, cur.coords.z),
            green,
        );
        prev = cur;
    }
    rg.render().unwrap();
}
