use crate::{
    behavior::{Action, Behavior},
    strategy::Context,
};
use nameof::name_of_type;

/// Execute `child` for at most `limit` seconds, then return.
pub struct TimeLimit {
    limit: f32,
    child: Box<Behavior>,
    start: Option<f32>,
}

impl TimeLimit {
    pub fn new(limit: f32, child: impl Behavior + 'static) -> Self {
        Self {
            limit,
            child: Box::new(child),
            start: None,
        }
    }
}

impl Behavior for TimeLimit {
    fn name(&self) -> &str {
        name_of_type!(TimeLimit)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let now = ctx.packet.GameInfo.TimeSeconds;
        let start = *self.start.get_or_insert(now);
        let elapsed = now - start;
        if elapsed >= self.limit {
            Action::Abort
        } else {
            self.child.execute2(ctx)
        }
    }
}
