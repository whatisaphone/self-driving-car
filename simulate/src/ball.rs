/// This module contains a rough transliteration of Sam Mish's ball prediction code.
///
/// Source:
/// https://github.com/samuelpmish/Lobot/blob/0d5b058e575493cf0bf34da78270da62e71a80d1/inc/Ball.h
use nalgebra::Vector3;

const BALL_RADIUS: f32 = 91.25;

pub struct Ball {
    #[allow(dead_code)]
    count: i32,
    x: Vector3<f32>,
    v: Vector3<f32>,
    w: Vector3<f32>,
}

impl Ball {
    pub fn new(x: Vector3<f32>, v: Vector3<f32>, w: Vector3<f32>) -> Ball {
        Ball { count: 0, x, v, w }
    }

    pub fn loc(&self) -> Vector3<f32> {
        self.x
    }

    pub fn vel(&self) -> Vector3<f32> {
        self.v
    }

    pub fn ang_vel(&self) -> Vector3<f32> {
        self.w
    }

    pub fn step(&mut self, dt: f32) {
        const R: f32 = BALL_RADIUS;
        const G: f32 = -650.0;
        const A: f32 = 0.0003;
        const Y: f32 = 2.0;
        const MU: f32 = 0.280;
        const C_R: f32 = 0.6;
        const DRAG: f32 = -0.0305;
        const W_MAX: f32 = 6.0;

        let a = DRAG * self.v + Vector3::new(0.0, 0.0, G);
        let v_pred = self.v + a * dt;
        let x_pred = self.x + v_pred * dt;
        let w_pred = self.w;

        if pitch_is_in_contact_with(x_pred, BALL_RADIUS) {
            let n = pitch_contact_normal();

            let v_perp = self.v.dot(&n) * n;
            let v_para = self.v - v_perp;
            let v_spin = R * n.cross(&self.w);
            let s = v_para + v_spin;

            let ratio = v_perp.norm() / s.norm();

            let delta_v_perp = -(1.0 + C_R) * v_perp;
            let delta_v_para = -f32::min(1.0, Y * ratio) * MU * s;

            self.w = w_pred + A * R * delta_v_para.cross(&n);
            self.v = v_pred + delta_v_perp + delta_v_para;

            // TODO
            self.x += 0.5 * (self.v + v_pred) * dt;
        } else {
            self.w = w_pred;
            self.v = v_pred;
            self.x = x_pred;
        }

        self.w *= f32::min(1.0, W_MAX / self.w.norm());
    }
}

fn pitch_is_in_contact_with(sphere_loc: Vector3<f32>, radius: f32) -> bool {
    return sphere_loc.z < radius;
}

fn pitch_contact_normal() -> Vector3<f32> {
    return Vector3::new(0.0, 0.0, 1.0);
}

#[cfg(test)]
mod test {
    use ball::Ball;
    use nalgebra::Vector3;
    use std::time::Instant;

    #[test]
    fn what() {
        const DT: f32 = 1.0 / 60.0;

        let start = Instant::now();
        for _ in 0..100 {
            let mut ball = Ball::new(
                Vector3::new(0.0, 0.0, 1000.0),
                Vector3::new(-50.0, 0.0, 0.0),
                Vector3::new(-200.0, 0.0, 200.0),
            );
            for x in 0..500 {
                ball.step(DT);
            }
        }
        let stop = Instant::now();
        println!("{:?}", (stop - start).subsec_millis());
    }
}
