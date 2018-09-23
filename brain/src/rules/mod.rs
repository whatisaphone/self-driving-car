use behavior::Action;
use nalgebra::Vector2;
use strategy::Context;
use utils::{ExtendPhysics, ExtendVector3};

pub struct SameBallTrajectory {
    prev_vel: Option<Vector2<f32>>,
}

impl SameBallTrajectory {
    pub fn new() -> SameBallTrajectory {
        SameBallTrajectory { prev_vel: None }
    }

    pub fn execute(&mut self, ctx: &mut Context) -> Option<Action> {
        if self.eval_vel_changed(ctx) {
            Some(Action::Abort)
        } else {
            None
        }
    }

    pub fn eval_vel_changed(&mut self, ctx: &mut Context) -> bool {
        let vel = ctx.packet.GameBall.Physics.vel().to_2d();
        let mut result = false;
        if let Some(prev_vel) = self.prev_vel {
            if (vel - prev_vel).norm() >= 10.0 {
                ctx.eeg.log("[SameBallTrajectory] Change detected");
                result = true;
            }
        }
        self.prev_vel = Some(vel);
        result
    }
}
