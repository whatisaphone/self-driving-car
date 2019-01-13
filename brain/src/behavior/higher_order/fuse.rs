use crate::strategy::{Action, Behavior, Context};
use nameof::name_of_type;

/// Run `child` until it returns, then do nothing forever.
pub struct Fuse {
    child: Option<Box<dyn Behavior>>,
}

impl Fuse {
    pub fn new(child: Box<dyn Behavior>) -> Self {
        Self { child: Some(child) }
    }
}

impl Behavior for Fuse {
    fn name(&self) -> &str {
        name_of_type!(Fuse)
    }

    fn execute_old(&mut self, _ctx: &mut Context<'_>) -> Action {
        // `take()` leaves a None behind, so this can only match `Some` once.
        match self.child.take() {
            Some(b) => Action::TailCall(b),
            None => Action::Yield(Default::default()),
        }
    }
}
