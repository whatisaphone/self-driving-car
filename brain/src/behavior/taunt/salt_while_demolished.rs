use crate::strategy::{Action, Behavior, Context, Priority};
use nameof::name_of_type;

pub struct SaltWhileDemolished;

impl SaltWhileDemolished {
    pub fn new() -> Self {
        Self
    }
}

impl Behavior for SaltWhileDemolished {
    fn name(&self) -> &str {
        name_of_type!(SaltWhileDemolished)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if !ctx.me().Demolished {
            return Action::Return;
        }

        if ctx.packet.GameInfo.TimeSeconds - *ctx.last_quick_chat >= 0.75 {
            ctx.eeg
                .quick_chat(rlbot::flat::QuickChatSelection::Reactions_Okay);
            *ctx.last_quick_chat = ctx.packet.GameInfo.TimeSeconds;
        }

        Action::Yield(Default::default())
    }
}
