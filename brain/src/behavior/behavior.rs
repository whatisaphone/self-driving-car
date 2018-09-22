use eeg::EEG;
use rlbot;
use strategy::Context;

pub trait Behavior: Send {
    fn name(&self) -> &'static str;

    fn priority(&self) -> Priority {
        Priority::Idle
    }

    // Soon to be deprecated
    fn capture(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Option<Action> {
        None
    }

    // Soon to be deprecated
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        unimplemented!("deprecated, use execute2 instead");
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        // Back-compat shim
        self.capture(ctx.packet, ctx.eeg)
            .unwrap_or_else(|| self.execute(ctx.packet, ctx.eeg))
    }
}

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
pub enum Priority {
    Idle,
    Save,
}

pub enum Action {
    Yield(rlbot::PlayerInput),
    Call(Box<Behavior>),
    Return,
}

impl Action {
    pub fn call(behavior: impl Behavior + 'static) -> Self {
        Action::Call(Box::new(behavior))
    }
}
