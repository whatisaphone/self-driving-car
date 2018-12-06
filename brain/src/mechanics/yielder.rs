use crate::{
    behavior::{Action, Behavior},
    strategy::Context,
};
use rlbot;

#[derive(new)]
pub struct Yielder {
    input: rlbot::ffi::PlayerInput,
    duration: f32,
    #[new(value = "None")]
    start: Option<f32>,
}

impl Behavior for Yielder {
    fn name(&self) -> &str {
        stringify!(Yielder)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let now = ctx.packet.GameInfo.TimeSeconds;
        let start = *self.start.get_or_insert(now);
        let elapsed = now - start;
        if elapsed < self.duration {
            Action::Yield(self.input)
        } else {
            Action::Return
        }
    }
}
