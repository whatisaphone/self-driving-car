use chip::Ball;
use nalgebra::{Point3, Vector3};
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
    pub fn predict(loc: Point3<f32>, vel: Vector3<f32>, ang_vel: Vector3<f32>) -> Self {
        let mut ball = Ball::new();
        ball.set_pos(loc);
        ball.set_vel(vel);
        ball.set_omega(ang_vel);

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

    /// Iterate over the frames, but skip the given number of seconds at the
    /// start.
    pub fn iter_delayed(&self, delay: f32) -> impl DoubleEndedIterator<Item = &BallFrame> {
        let delay_frames = (delay / DT) as usize;
        self.frames.iter().skip(delay_frames)
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
