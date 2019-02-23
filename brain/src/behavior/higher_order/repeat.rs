use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context},
};

/// Run `behavior` forever
pub struct Repeat<B, F>
where
    B: Behavior,
    F: Fn() -> B + Send,
{
    factory: F,
    current: B,
}

impl<B, F> Repeat<B, F>
where
    B: Behavior,
    F: Fn() -> B + Send,
{
    pub fn new(factory: F) -> Self {
        let current = factory();
        Self { factory, current }
    }
}

impl<B, F> Behavior for Repeat<B, F>
where
    B: Behavior,
    F: Fn() -> B + Send,
{
    fn name(&self) -> &str {
        stringify!(Repeat)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg
            .draw(Drawable::print(self.current.blurb(), color::YELLOW));
        match self.current.execute_old(ctx) {
            Action::Yield(i) => Action::Yield(i),
            Action::TailCall(b) => Action::TailCall(b),
            Action::RootCall(i) => Action::RootCall(i),
            Action::Return | Action::Abort => {
                ctx.eeg.log(self.name(), "repeating");
                self.current = (self.factory)();
                Action::Yield(Default::default())
            }
        }
    }
}
