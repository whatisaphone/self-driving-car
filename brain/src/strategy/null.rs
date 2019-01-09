use crate::{
    behavior::higher_order::NullBehavior,
    strategy::{strategy::Strategy, Behavior, Context},
};
use derive_new::new;

#[derive(new)]
pub struct NullStrategy;

impl Strategy for NullStrategy {
    fn baseline(&mut self, _ctx: &mut Context<'_>) -> Box<dyn Behavior> {
        Box::new(NullBehavior::new())
    }

    fn interrupt(
        &mut self,
        _ctx: &mut Context<'_>,
        _current: &dyn Behavior,
    ) -> Option<Box<dyn Behavior>> {
        None
    }
}
