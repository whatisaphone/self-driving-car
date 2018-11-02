use routing::models::{CarState, SegmentPlan, SegmentRunAction, SegmentRunner};
use std::collections::VecDeque;
use strategy::Context;

#[derive(Clone, new)]
pub struct Chain {
    segments: Vec<Box<SegmentPlan>>,
}

impl SegmentPlan for Chain {
    fn start(&self) -> CarState {
        self.segments.first().unwrap().start()
    }

    fn end(&self) -> CarState {
        self.segments.last().unwrap().end()
    }

    fn duration(&self) -> f32 {
        self.segments.iter().map(|s| s.duration()).sum()
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(Chainer::new(
            self.segments.iter().map(|s| s.run()).collect(),
        ))
    }

    fn draw(&self, ctx: &mut Context) {
        for segment in &self.segments {
            segment.draw(ctx);
        }
    }
}

#[derive(new)]
struct Chainer {
    segments: VecDeque<Box<SegmentRunner>>,
}

impl SegmentRunner for Chainer {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let action = self.segments.front_mut().unwrap().execute(ctx);
        match action {
            SegmentRunAction::Yield(i) => return SegmentRunAction::Yield(i),
            SegmentRunAction::Success => true,
            SegmentRunAction::Failure => return SegmentRunAction::Failure,
        };

        self.segments.pop_front().unwrap();
        if self.segments.is_empty() {
            SegmentRunAction::Success
        } else {
            ctx.eeg.log("[Chainer] next runner");
            self.execute(ctx)
        }
    }
}
