use eeg::EEG;
use rlbot;
use strategy::Context;

pub trait Behavior: Send {
    fn name(&self) -> &str;

    fn priority(&self) -> Priority {
        Priority::Idle
    }

    // Soon to be deprecated
    fn capture(&mut self, _packet: &rlbot::LiveDataPacket, _eeg: &mut EEG) -> Option<Action> {
        None
    }

    // Soon to be deprecated
    fn execute(&mut self, _packet: &rlbot::LiveDataPacket, _eeg: &mut EEG) -> Action {
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
    Force,
}

pub enum Action {
    Yield(rlbot::PlayerInput),
    Call(Box<Behavior>),
    Return,
    Abort,
}

impl Action {
    pub fn call(behavior: impl Behavior + 'static) -> Self {
        Action::Call(Box::new(behavior))
    }
}
