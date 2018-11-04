use behavior::{Action, Behavior, Chain, Priority};
use common::prelude::*;
use eeg::{color, Drawable};
use mechanics::Yielder;
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use simulate::CarForwardDodge1D;
use strategy::Context;

#[derive(Clone, new)]
pub struct ForwardDodge {
    start: CarState,
    dodge: CarForwardDodge1D,
}

impl SegmentPlan for ForwardDodge {
    fn name(&self) -> &str {
        stringify!(ForwardDodge)
    }

    fn start(&self) -> CarState {
        self.start.clone()
    }

    fn end(&self) -> CarState {
        CarState2D {
            loc: self.start.loc.to_2d() + self.start.vel.to_2d().normalize() * self.dodge.end_dist,
            rot: self.start.rot.to_2d(),
            vel: self.start.vel.to_2d().normalize() * self.dodge.end_speed,
            boost: self.start.boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        self.dodge.duration()
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(ForwardDodgeRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg.draw(Drawable::Line(
            self.start.loc.to_2d(),
            self.end().loc.to_2d(),
            color::GREEN,
        ));
    }
}

struct ForwardDodgeRunner {
    behavior: Box<Behavior>,
}

impl ForwardDodgeRunner {
    pub fn new(plan: ForwardDodge) -> Self {
        let behavior = Box::new(Chain::new(
            Priority::Idle,
            vec![
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Jump: true,
                        ..Default::default()
                    },
                    plan.dodge.jump_duration,
                )),
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        ..Default::default()
                    },
                    plan.dodge.wait_duration,
                )),
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Pitch: -1.0,
                        Jump: true,
                        ..Default::default()
                    },
                    1.0 / 120.0,
                )),
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        ..Default::default()
                    },
                    plan.dodge.dodge_duration - 1.0 / 120.0,
                )),
            ],
        ));
        Self { behavior }
    }
}

impl SegmentRunner for ForwardDodgeRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        match self.behavior.execute2(ctx) {
            Action::Yield(i) => SegmentRunAction::Yield(i),
            Action::Call(_) => panic!("Action::Call not supported in SegmentRunner"),
            Action::Return => SegmentRunAction::Success,
            Action::Abort => SegmentRunAction::Failure,
        }
    }
}
