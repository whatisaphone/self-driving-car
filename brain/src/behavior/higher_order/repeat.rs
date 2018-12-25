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

    fn execute(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg
            .draw(Drawable::print(self.current.name(), color::YELLOW));
        match self.current.execute(ctx) {
            Action::Yield(i) => Action::Yield(i),
            Action::Call(b) => Action::Call(b),
            Action::Return | Action::Abort => {
                ctx.eeg.log("[Repeat] repeating");
                self.current = (self.factory)();
                Action::Yield(Default::default())
            }
        }
    }
}
