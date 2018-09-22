use behavior::Action;
use eeg::{color, Drawable};
use nalgebra::Vector2;
use rlbot;
use strategy::Context;
use utils::{ExtendPhysics, ExtendVector3};
use EEG;

pub struct Finishable {
    finished: bool,
}

impl Finishable {
    pub fn new() -> Finishable {
        Finishable { finished: false }
    }

    pub fn execute(&mut self, _packet: &rlbot::LiveDataPacket, _eeg: &mut EEG) -> Option<Action> {
        if self.finished {
            return Some(Action::Return);
        }
        None
    }

    pub fn set_finished(&mut self) {
        self.finished = true;
    }
}

pub struct SameBallTrajectory {
    prev_vel: Option<Vector2<f32>>,
}

impl SameBallTrajectory {
    pub fn new() -> SameBallTrajectory {
        SameBallTrajectory { prev_vel: None }
    }

    pub fn execute(&mut self, ctx: &mut Context) -> Option<Action> {
        if self.eval_vel_changed(ctx) {
            Some(Action::Return)
        } else {
            None
        }
    }

    pub fn eval_vel_changed(&mut self, ctx: &mut Context) -> bool {
        let vel = ctx.packet.GameBall.Physics.vel().to_2d();
        let mut result = false;
        if let Some(prev_vel) = self.prev_vel {
            if (vel - prev_vel).norm() >= 10.0 {
                ctx.eeg
                    .draw(Drawable::print("Ball trajectory has changed", color::GREEN));
                result = true;
            }
        }
        self.prev_vel = Some(vel);
        result
    }
}
