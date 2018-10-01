use behavior::behavior::{Action, Behavior};
use rlbot;
use strategy::Context;

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
        Action::Yield(rlbot::PlayerInput::default())
    }
}
