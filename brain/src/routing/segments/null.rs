use crate::{
    routing::models::{CarState, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
};
use derive_new::new;
use nameof::name_of_type;

#[derive(Clone, new)]
pub struct NullSegment {
    start: CarState,
}

impl SegmentPlan for NullSegment {
    fn name(&self) -> &str {
        name_of_type!(NullSegment)
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

    fn run(&self) -> Box<dyn SegmentRunner> {
        Box::new(NullSegmentRunner::new())
    }

    fn draw(&self, _ctx: &mut Context<'_>) {}
}

#[derive(new)]
struct NullSegmentRunner;

impl SegmentRunner for NullSegmentRunner {
    fn name(&self) -> &str {
        name_of_type!(NullSegmentRunner)
    }

    fn execute(&mut self, _ctx: &mut Context<'_>) -> SegmentRunAction {
        SegmentRunAction::Success
    }
}
