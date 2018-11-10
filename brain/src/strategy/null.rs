use behavior::{Behavior, NullBehavior};
use strategy::{strategy::Strategy, Context};

#[derive(new)]
pub struct NullStrategy;

impl Strategy for NullStrategy {
    fn baseline(&mut self, _ctx: &mut Context) -> Box<Behavior> {
        Box::new(NullBehavior::new())
    }

    fn interrupt(&mut self, _ctx: &mut Context, _current: &Behavior) -> Option<Box<Behavior>> {
        None
    }
}
