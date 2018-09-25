use nalgebra::Vector3;
use rlbot;
use simulate::chip::Ball;
use utils::{ExtendPhysics, TotalF32};

const DT: f32 = 1.0 / 60.0;
const PREDICT_DURATION: f32 = 5.0;

pub struct BallTrajectory {
    frames: Vec<BallFrame>,
}

pub struct BallFrame {
    pub t: f32,
    pub loc: Vector3<f32>,
}

impl BallFrame {
    pub fn dt(&self) -> f32 {
        DT
    }
}

impl BallTrajectory {
    pub fn predict(packet: &rlbot::LiveDataPacket) -> Self {
        let mut ball = Ball::new(
            packet.GameBall.Physics.loc(),
            packet.GameBall.Physics.vel(),
            packet.GameBall.Physics.ang_vel(),
        );
        let mut frames = Vec::with_capacity((PREDICT_DURATION / DT).ceil() as usize);
        let mut t = 0.0;
        while t < PREDICT_DURATION {
            t += DT;
            ball.step(DT);
            frames.push(BallFrame { t, loc: ball.loc() });
        }

        Self { frames }
    }

    pub fn iter(&self) -> impl Iterator<Item = &BallFrame> {
        self.frames.iter()
    }

    pub fn at_time(&self, t: f32) -> Option<&BallFrame> {
        let i = match self
            .frames
            .binary_search_by_key(&TotalF32(t), |f| TotalF32(f.t))
        {
            Ok(i) => i,
            Err(i) => i,
        };
        if i >= self.frames.len() {
            return None;
        }
        Some(&self.frames[i])
    }
}
