use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};

/// Run `child` while `predicate` holds true.
pub struct While<P>
where
    P: Predicate,
{
    predicate: P,
    child: Box<dyn Behavior>,
}

pub trait Predicate: Send {
    fn name(&self) -> &str;
    fn evaluate(&mut self, ctx: &mut Context<'_>) -> bool;
}

impl<P> While<P>
where
    P: Predicate,
{
    pub fn new(predicate: P, child: impl Behavior + 'static) -> Self {
        Self {
            predicate,
            child: Box::new(child),
        }
    }
}

impl<P> Behavior for While<P>
where
    P: Predicate,
{
    fn name(&self) -> &str {
        stringify!(While)
    }

    fn priority(&self) -> Priority {
        self.child.priority()
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if !self.predicate.evaluate(ctx) {
            ctx.eeg.log(self.name(), "terminating");
            return Action::Return;
        }

        ctx.eeg
            .draw(Drawable::print(self.predicate.name(), color::GREEN));
        ctx.eeg
            .draw(Drawable::print(self.child.blurb(), color::YELLOW));

        match self.child.execute_old(ctx) {
            Action::Yield(i) => Action::Yield(i),
            Action::TailCall(b) => {
                // The tail-called behavior should not escape the predicate.
                self.child = b;
                self.execute_old(ctx)
            }
            Action::RootCall(b) => Action::RootCall(b),
            Action::Return => Action::Return,
            Action::Abort => Action::Abort,
        }
    }
}
