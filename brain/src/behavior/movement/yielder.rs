use crate::strategy::{Action, Behavior, Context};
use derive_new::new;
use nameof::name_of_type;

#[derive(new)]
pub struct Yielder {
    input: rlbot::ffi::PlayerInput,
    duration: f32,
    #[new(value = "None")]
    start: Option<f32>,
}

impl Behavior for Yielder {
    fn name(&self) -> &str {
        name_of_type!(Yielder)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
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
