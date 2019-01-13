use crate::{
    routing::models::{CarState, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
};
use derive_new::new;
use nameof::name_of_type;
use std::collections::VecDeque;

#[derive(Clone, new)]
pub struct Chain {
    segments: Vec<Box<dyn SegmentPlan>>,
}

impl SegmentPlan for Chain {
    fn name(&self) -> &str {
        name_of_type!(Chain)
    }

    fn start(&self) -> CarState {
        self.segments.first().unwrap().start()
    }

    fn end(&self) -> CarState {
        self.segments.last().unwrap().end()
    }

    fn duration(&self) -> f32 {
        self.segments.iter().map(|s| s.duration()).sum()
    }

    fn run(&self) -> Box<dyn SegmentRunner> {
        Box::new(Chainer::new(
            self.segments.iter().map(|s| s.run()).collect(),
        ))
    }

    fn draw(&self, ctx: &mut Context<'_>) {
        for segment in &self.segments {
            segment.draw(ctx);
        }
    }
}

#[derive(new)]
struct Chainer {
    segments: VecDeque<Box<dyn SegmentRunner>>,
}

impl SegmentRunner for Chainer {
    fn name(&self) -> &str {
        name_of_type!(Chainer)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> SegmentRunAction {
        match self.segments.front_mut().unwrap().execute_old(ctx) {
            SegmentRunAction::Yield(i) => return SegmentRunAction::Yield(i),
            SegmentRunAction::Success => true,
            SegmentRunAction::Failure => return SegmentRunAction::Failure,
        };

        self.segments.pop_front().unwrap();
        if self.segments.is_empty() {
            SegmentRunAction::Success
        } else {
            ctx.eeg.log(self.name(), "next runner");
            self.execute_old(ctx)
        }
    }
}
