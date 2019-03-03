use crate::strategy::{Action, Behavior, Context, Priority};
use nameof::name_of_type;

pub struct Yielder {
    input: common::halfway_house::PlayerInput,
    duration: f32,
    priority: Priority,
    start: Option<f32>,
}

impl Yielder {
    pub fn new(duration: f32, input: common::halfway_house::PlayerInput) -> Self {
        Self {
            input,
            duration,
            priority: Priority::Idle,
            start: None,
        }
    }

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
