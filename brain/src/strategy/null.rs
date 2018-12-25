use crate::{
    behavior::{higher_order::NullBehavior, Behavior},
    strategy::{strategy::Strategy, Context},
};
use derive_new::new;

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
