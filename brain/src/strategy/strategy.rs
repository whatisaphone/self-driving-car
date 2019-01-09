use crate::strategy::{Behavior, Context};

pub trait Strategy: Send {
    fn baseline(&mut self, ctx: &mut Context<'_>) -> Box<dyn Behavior>;
    fn interrupt(
        &mut self,
        ctx: &mut Context<'_>,
        current: &dyn Behavior,
    ) -> Option<Box<dyn Behavior>>;
}
