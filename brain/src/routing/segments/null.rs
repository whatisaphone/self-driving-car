use crate::{
    routing::models::{CarState, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
};

#[derive(Clone, new)]
pub struct NullSegment {
    start: CarState,
}

impl SegmentPlan for NullSegment {
    fn name(&self) -> &str {
        stringify!(NullSegment)
    }

    fn start(&self) -> CarState {
        self.start.clone()
    }

    fn end(&self) -> CarState {
        self.start.clone()
    }

    fn duration(&self) -> f32 {
        0.0
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(NullSegmentRunner::new())
    }

    fn draw(&self, _ctx: &mut Context) {}
}

#[derive(new)]
struct NullSegmentRunner;

impl SegmentRunner for NullSegmentRunner {
    fn execute(&mut self, _ctx: &mut Context) -> SegmentRunAction {
        SegmentRunAction::Success
    }
}
