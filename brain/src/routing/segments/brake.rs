use common::{prelude::*, rl};
use eeg::{color, Drawable};
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use strategy::Context;

#[derive(Clone)]
pub struct Brake {
    start: CarState2D,
    target_speed: f32,
}

impl Brake {
    pub fn new(start: CarState2D, mut target_speed: f32) -> Self {
        let start_speed = start.vel.norm();
        if target_speed >= start_speed {
            warn!("[Brake] target speed is faster than start speed");
            target_speed = start_speed;
        }

        Self {
            start,
            target_speed,
        }
    }
}

impl SegmentPlan for Brake {
    fn name(&self) -> &str {
        stringify!(Brake)
    }

    fn start(&self) -> CarState {
        self.start.to_3d()
    }

    fn end(&self) -> CarState {
        // This is a pretty rough estimate. I'm lazy so hopefully it doesn't make too
        // much of a difference.
        let mean_speed = 1.0 / ((1.0 / self.start.vel.norm() + 1.0 / self.target_speed) / 2.0);
        let dist = mean_speed * self.duration();
        CarState2D {
            loc: self.start.loc + self.start.forward_axis().as_ref() * dist,
            rot: self.start.rot,
            vel: self.start.vel.normalize() * self.target_speed,
            boost: self.start.boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        // XXX: This is basically just a wild guess.
        let start_speed = self.start.vel.norm();
        (start_speed - self.target_speed) * (1.0 / rl::CAR_MAX_SPEED)
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(Braker::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg.draw(Drawable::Line(
            self.start.loc,
            self.end().loc.to_2d(),
            color::RED,
        ));
    }
}

#[derive(new)]
struct Braker {
    plan: Brake,
}

impl SegmentRunner for Braker {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let speed = ctx.me().Physics.vel().to_2d().norm();
        if speed < self.plan.target_speed {
            return SegmentRunAction::Success;
        }

        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: -1.0,
            ..Default::default()
        })
    }
}
