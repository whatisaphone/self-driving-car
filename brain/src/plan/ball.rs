use chip::Ball;
use common::prelude::*;
use nalgebra::{Point3, Vector3};
use std::mem;
use utils::TotalF32;

const DT: f32 = 1.0 / 120.0;
const PREDICT_DURATION: f32 = 5.0;

pub struct BallTrajectory {
    frames: Vec<BallFrame>,
}

pub struct BallFrame {
    pub t: f32,
    pub loc: Point3<f32>,
    pub vel: Vector3<f32>,
}

impl BallFrame {
    pub fn dt(&self) -> f32 {
        DT
    }
}

impl BallTrajectory {
    /// Return the starting frame of the prediction (e.g., where the ball is
    /// right now).
    pub fn start(&self) -> &BallFrame {
        self.frames.first().unwrap()
    }

    pub fn iter(&self) -> impl DoubleEndedIterator<Item = &BallFrame> {
        self.frames.iter()
    }

    /// Iterate over the frames, but skip the given number of seconds at the
    /// start.
    pub fn iter_delayed(&self, delay: f32) -> impl DoubleEndedIterator<Item = &BallFrame> {
        let delay_frames = (delay / DT) as usize;
        self.frames.iter().skip(delay_frames)
    }

    pub fn hacky_expensive_slice(&self, delay: f32) -> Self {
        let start = self.start();
        Self {
            frames: self
                .iter_delayed(delay)
                .map(|f| BallFrame {
                    t: f.t - start.t,
                    ..*f
                })
                .collect(),
        }
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

pub trait BallPredictor {
    fn predict(&self, packet: &rlbot::ffi::LiveDataPacket) -> BallTrajectory;
}

#[derive(new)]
pub struct ChipBallPrediction;

impl BallPredictor for ChipBallPrediction {
    fn predict(&self, packet: &rlbot::ffi::LiveDataPacket) -> BallTrajectory {
        let mut ball = Ball::new();
        ball.set_pos(packet.GameBall.Physics.locp());
        ball.set_vel(packet.GameBall.Physics.vel());
        ball.set_omega(packet.GameBall.Physics.ang_vel());

        let num_frames = (PREDICT_DURATION / DT).ceil() as usize;
        let mut frames = Vec::with_capacity(num_frames);
        let mut t = 0.0;

        // Include the initial frame to allow interpolation when the framerate is
        // faster than `DT`.
        frames.push(BallFrame {
            t,
            loc: ball.pos(),
            vel: ball.vel(),
        });

        while frames.len() < num_frames {
            t += DT;
            ball.step(DT);
            frames.push(BallFrame {
                t,
                loc: ball.pos(),
                vel: ball.vel(),
            });
        }

        BallTrajectory { frames }
    }
}

#[derive(new)]
pub struct FrameworkBallPrediction {
    rlbot: &'static rlbot::RLBot,
}

impl BallPredictor for FrameworkBallPrediction {
    fn predict(&self, _packet: &rlbot::ffi::LiveDataPacket) -> BallTrajectory {
        let mut packet: rlbot::ffi::BallPredictionPacket = unsafe { mem::uninitialized() };
        self.rlbot.get_ball_prediction_struct(&mut packet).unwrap();
        let start_time = packet.Slice[0].GameSeconds;
        let frames = packet
            .Slice
            .iter()
            .take(packet.NumSlices as usize)
            .map(|slice| BallFrame {
                t: slice.GameSeconds - start_time,
                loc: point3(slice.Physics.Location),
                vel: vector3(slice.Physics.Velocity),
            })
            .collect();
        BallTrajectory { frames }
    }
}

fn point3(v: rlbot::ffi::Vector3) -> Point3<f32> {
    Point3::new(v.X, v.Y, v.Z)
}

fn vector3(v: rlbot::ffi::Vector3) -> Vector3<f32> {
    Vector3::new(v.X, v.Y, v.Z)
}
