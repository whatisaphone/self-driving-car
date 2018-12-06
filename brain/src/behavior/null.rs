use crate::{
    behavior::behavior::{Action, Behavior},
    strategy::Context,
};
use rlbot;

#[allow(dead_code)]
pub struct NullBehavior;

impl NullBehavior {
    #[allow(dead_code)]
    pub fn new() -> NullBehavior {
        NullBehavior
    }
}

impl Behavior for NullBehavior {
    fn name(&self) -> &str {
        stringify!(NullBehavior)
    }

    fn execute2(&mut self, _ctx: &mut Context) -> Action {
        Action::Yield(rlbot::ffi::PlayerInput::default())
    }
}
