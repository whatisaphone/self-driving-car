use crate::utils::TotalF32;
use chip::Ball;
use common::{math::fractionality, prelude::*, rl};
use nalgebra::{Point3, Vector3};
use std::{iter::Cloned, mem, slice::Iter};

const PREDICT_DURATION: f32 = 7.0;

pub struct BallTrajectory {
    frames: Vec<BallFrame>,
}

#[derive(Clone)]
pub struct BallFrame {
    pub t: f32,
    dt: f32,
    pub loc: Point3<f32>,
    pub vel: Vector3<f32>,
}

impl BallFrame {
    pub fn dt(&self) -> f32 {
        self.dt
    }
}

impl BallTrajectory {
    fn new(frames: Vec<BallFrame>) -> Self {
        assert!(!frames.is_empty());
        Self { frames }
    }

    /// Return the starting frame of the prediction (e.g., where the ball is
    /// right now).
    pub fn start(&self) -> &BallFrame {
        self.frames.first().unwrap()
    }

    pub fn iter(&self) -> impl DoubleEndedIterator<Item = &BallFrame> {
        self.frames.iter()
    }

    pub fn iter_step_by<'a>(&'a self, dt: f32) -> impl Iterator<Item = BallFrame> + 'a {
        let factor = dt / self.frames[0].dt;
        // Enforce integral divisions.
        assert!(fractionality(factor) <= 1e-6);
        let factor = factor.round();

        self.frames
            .iter()
            .step_by(factor as usize)
            .map(move |f| BallFrame { dt, ..*f })
    }

    /// Iterate over the frames, but skip the given number of seconds at the
    /// start.
    pub fn hacky_expensive_slice(&self, delay: f32) -> Self {
        let frames = match self.frames.iter().position(|f| f.t >= delay) {
            Some(i) => {
                let start_time = self.frames[i].t;
                self.frames[i..]
                    .iter()
                    .map(|f| BallFrame {
                        t: f.t - start_time,
                        ..*f
                    })
                    .collect()
            }
            None => vec![self.frames.last().unwrap().clone()],
        };

        Self::new(frames)
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

impl<'a> IntoIterator for &'a BallTrajectory {
    type Item = BallFrame;
    type IntoIter = Cloned<Iter<'a, BallFrame>>;

    fn into_iter(self) -> Self::IntoIter {
        self.frames.iter().cloned()
    }
}

pub trait BallPredictor {
    fn predict(&self, packet: &rlbot::ffi::LiveDataPacket) -> BallTrajectory;
}

#[derive(new)]
pub struct ChipBallPrediction;

impl BallPredictor for ChipBallPrediction {
    fn predict(&self, packet: &rlbot::ffi::LiveDataPacket) -> BallTrajectory {
        const DT: f32 = rl::PHYSICS_DT;

        let mut ball = Ball::new();
        ball.set_pos(packet.GameBall.Physics.loc());
        ball.set_vel(packet.GameBall.Physics.vel());
        ball.set_omega(packet.GameBall.Physics.ang_vel());

        let num_frames = (PREDICT_DURATION / DT).ceil() as usize;
        let mut frames = Vec::with_capacity(num_frames);
        let mut t = 0.0;

        // Include the initial frame to allow interpolation when the framerate is
        // faster than `DT`.
        frames.push(BallFrame {
            t,
            dt: DT,
            loc: ball.pos(),
            vel: ball.vel(),
        });

        while frames.len() < num_frames {
            t += DT;
            ball.step(DT);
            frames.push(BallFrame {
                t,
                dt: DT,
                loc: ball.pos(),
                vel: ball.vel(),
            });
        }

        BallTrajectory::new(frames)
    }
}

#[derive(new)]
pub struct FrameworkBallPrediction {
    rlbot: &'static rlbot::RLBot,
}

impl BallPredictor for FrameworkBallPrediction {
    fn predict(&self, _packet: &rlbot::ffi::LiveDataPacket) -> BallTrajectory {
        const DT: f32 = 1.0 / 60.0;

        let mut packet: rlbot::ffi::BallPredictionPacket = unsafe { mem::uninitialized() };
        self.rlbot.get_ball_prediction_struct(&mut packet).unwrap();
        let start_time = packet.Slice[0].GameSeconds;
        let frames = packet
            .Slice
            .iter()
            .take(packet.NumSlices as usize)
            .map(|slice| BallFrame {
                t: slice.GameSeconds - start_time,
                dt: DT,
                loc: point3(slice.Physics.Location),
                vel: vector3(slice.Physics.Velocity),
            })
            .collect();
        BallTrajectory::new(frames)
    }
}

fn point3(v: rlbot::ffi::Vector3) -> Point3<f32> {
    Point3::new(v.X, v.Y, v.Z)
}

fn vector3(v: rlbot::ffi::Vector3) -> Vector3<f32> {
    Vector3::new(v.X, v.Y, v.Z)
}
