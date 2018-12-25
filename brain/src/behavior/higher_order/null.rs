use crate::strategy::{Action, Behavior, Context};
use nameof::name_of_type;
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
        name_of_type!(NullBehavior)
    }

    fn execute(&mut self, _ctx: &mut Context) -> Action {
        Action::Yield(rlbot::ffi::PlayerInput::default())
    }
}
