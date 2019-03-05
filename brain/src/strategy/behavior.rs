use crate::strategy::Context;

pub trait Behavior: Send {
    /// A very short string identifying the behavior; usually just the name of
    /// the object.
    fn name(&self) -> &str;

    /// A short string identifying the behavior in one line.
    fn blurb(&self) -> &str {
        self.name()
    }

    fn priority(&self) -> Priority {
        Priority::Idle
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action;
}

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
pub enum Priority {
    Idle,
    Defense,
    Save,
    Strike,
    Taunt,
    Force,
}

pub enum Action {
    Yield(common::halfway_house::PlayerInput),
    TailCall(Box<dyn Behavior>),
    RootCall(Box<dyn Behavior>),
    Return,
    Abort,
}

impl Action {
    pub fn tail_call(behavior: impl Behavior + 'static) -> Self {
        Action::TailCall(Box::new(behavior))
    }
}
