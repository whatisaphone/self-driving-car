use crate::strategy::{Action, Behavior, Context, Priority};
use derive_new::new;
use nameof::name_of_type;

#[derive(new)]
pub struct Yielder {
    input: common::halfway_house::PlayerInput,
    duration: f32,
    #[new(value = "Priority::Idle")]
    priority: Priority,
    #[new(value = "None")]
    start: Option<f32>,
}

impl Yielder {
    #[allow(dead_code)]
    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }
}

impl Behavior for Yielder {
    fn name(&self) -> &str {
        name_of_type!(Yielder)
    }

    fn priority(&self) -> Priority {
        self.priority
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
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
