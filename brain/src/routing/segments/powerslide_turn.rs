use common::{
    ext::{ExtendUnitVector2, ExtendVector2},
    physics::CAR_LOCAL_FORWARD_AXIS_2D,
};
use eeg::{color, Drawable};
use rlbot;
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use simulate::CarPowerslideTurnPlan;
use strategy::Context;

#[derive(Clone)]
pub struct PowerslideTurn {
    plan: CarPowerslideTurnPlan,
    boost: f32,
}

impl PowerslideTurn {
    pub fn new(plan: CarPowerslideTurnPlan, boost: f32) -> Self {
        Self { plan, boost }
    }
}

impl SegmentPlan for PowerslideTurn {
    fn start(&self) -> CarState {
        CarState2D {
            loc: self.plan.start_loc,
            rot: CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&self.plan.start_vel.to_axis()),
            vel: self.plan.start_vel,
            boost: self.boost,
        }
        .to_3d()
    }

    fn end(&self) -> CarState {
        CarState2D {
            loc: self.plan.end_loc,
            rot: self.plan.end_rot,
            vel: self.plan.end_vel,
            boost: self.boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        self.plan.duration
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(PowerslideTurnRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg.draw(Drawable::Line(
            self.plan.start_loc,
            self.plan.end_loc,
            color::RED,
        ));
    }
}

struct PowerslideTurnRunner {
    plan: PowerslideTurn,
    start_time: Option<f32>,
}

impl PowerslideTurnRunner {
    fn new(plan: PowerslideTurn) -> Self {
        Self {
            plan,
            start_time: None,
        }
    }
}

impl SegmentRunner for PowerslideTurnRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let now = ctx.packet.GameInfo.TimeSeconds;
        let start_time = *self.start_time.get_or_insert(now);
        let elapsed = now - start_time;
        if elapsed >= self.plan.plan.duration {
            return SegmentRunAction::Success;
        }

        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: self.plan.plan.throttle,
            Steer: self.plan.plan.steer,
            Handbrake: true,
            ..Default::default()
        })
    }
}
