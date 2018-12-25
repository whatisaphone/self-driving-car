use crate::strategy::{Action, Behavior, Context};
use nameof::name_of_type;

/// Run `child` until it returns, then do nothing forever.
pub struct Fuse {
    child: Option<Box<Behavior>>,
}

impl Fuse {
    pub fn new(child: Box<Behavior>) -> Self {
        Self { child: Some(child) }
    }
}

impl Behavior for Fuse {
    fn name(&self) -> &str {
        name_of_type!(Fuse)
    }

    fn execute2(&mut self, _ctx: &mut Context) -> Action {
        // `take()` leaves a None behind, so this can only match `Some` once.
        match self.child.take() {
            Some(b) => Action::Call(b),
            None => Action::Yield(Default::default()),
        }
    }
}
