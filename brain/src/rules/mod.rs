use arrayvec::ArrayVec;
use behavior::Action;
use common::prelude::*;
use nalgebra::Vector3;
use simulate::linear_interpolate;
use strategy::Context;

const NUM_PREDICTION_FRAMES: usize = 8;
const ERROR_THRESHOLD: f32 = 5.0;

/// Track the ball's trajectory vs. our prediction, and if they differ by too
/// much, abort.
pub struct SameBallTrajectory {
    predict_t: ArrayVec<[f32; NUM_PREDICTION_FRAMES]>,
    predict_x: ArrayVec<[f32; NUM_PREDICTION_FRAMES]>,
    predict_y: ArrayVec<[f32; NUM_PREDICTION_FRAMES]>,
    predict_z: ArrayVec<[f32; NUM_PREDICTION_FRAMES]>,
}

impl SameBallTrajectory {
    pub fn new() -> SameBallTrajectory {
        SameBallTrajectory {
            predict_t: ArrayVec::new(),
            predict_x: ArrayVec::new(),
            predict_y: ArrayVec::new(),
            predict_z: ArrayVec::new(),
        }
    }

    pub fn execute(&mut self, ctx: &mut Context) -> Option<Action> {
        if self.eval_vel_changed(ctx) {
            Some(Action::Abort)
        } else {
            self.update_snapshot(ctx);
            None
        }
    }

    fn update_snapshot(&mut self, ctx: &mut Context) {
        self.predict_t.clear();
        self.predict_x.clear();
        self.predict_y.clear();
        self.predict_z.clear();

        for frame in ctx
            .scenario
            .ball_prediction()
            .iter()
            .take(NUM_PREDICTION_FRAMES)
        {
            self.predict_t
                .push(ctx.packet.GameInfo.TimeSeconds + frame.t);
            self.predict_x.push(frame.loc.x);
            self.predict_y.push(frame.loc.y);
            self.predict_z.push(frame.loc.z);
        }
    }

    fn eval_vel_changed(&mut self, ctx: &mut Context) -> bool {
        if self.predict_t.is_empty() {
            return false;
        }

        if ctx.packet.GameInfo.TimeSeconds <= *self.predict_t.first().unwrap()
            || ctx.packet.GameInfo.TimeSeconds > *self.predict_t.last().unwrap()
        {
            warn!("game time not in prediction range");
        }

        let predicted_x = linear_interpolate(
            &self.predict_t[..],
            &self.predict_x[..],
            ctx.packet.GameInfo.TimeSeconds,
        );
        let predicted_y = linear_interpolate(
            &self.predict_t[..],
            &self.predict_y[..],
            ctx.packet.GameInfo.TimeSeconds,
        );
        let predicted_z = linear_interpolate(
            &self.predict_t[..],
            &self.predict_z[..],
            ctx.packet.GameInfo.TimeSeconds,
        );
        let predicted_loc = Vector3::new(predicted_x, predicted_y, predicted_z);

        let actual_loc = ctx.packet.GameBall.Physics.loc();
        let error = (predicted_loc - actual_loc).norm();
        if error >= ERROR_THRESHOLD {
            ctx.eeg.log(format!(
                "[SameBallTrajectory] perturbance detected with error {:.2}",
                error,
            ));
            true
        } else {
            false
        }
    }
}
