use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};

/// Run `child` while `predicate` holds true.
pub struct While<P, B>
where
    P: Predicate,
    B: Behavior,
{
    predicate: P,
    child: B,
}

pub trait Predicate: Send {
    fn name(&self) -> &str;
    fn evaluate(&mut self, ctx: &mut Context) -> bool;
}

impl<P, B> While<P, B>
where
    P: Predicate,
    B: Behavior,
{
    pub fn new(predicate: P, child: B) -> Self {
        Self { predicate, child }
    }
}

impl<P, B> Behavior for While<P, B>
where
    P: Predicate,
    B: Behavior,
{
    fn name(&self) -> &str {
        stringify!(While)
    }

    fn priority(&self) -> Priority {
        self.child.priority()
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        if !self.predicate.evaluate(ctx) {
            ctx.eeg.log("[While] Terminating");
            return Action::Abort;
        }

        ctx.eeg
            .draw(Drawable::print(self.child.name(), color::YELLOW));

        self.child.execute(ctx)
    }
}
