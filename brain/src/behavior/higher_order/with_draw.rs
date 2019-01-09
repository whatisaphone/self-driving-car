use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};

pub struct WithDraw<B: Behavior> {
    draw: Vec<Drawable>,
    behavior: B,
}

impl<B: Behavior> WithDraw<B> {
    pub fn new(draw: Vec<Drawable>, behavior: B) -> Self {
        Self { draw, behavior }
    }
}

impl<B: Behavior> Behavior for WithDraw<B> {
    fn name(&self) -> &str {
        stringify!(WithDraw)
    }

    fn priority(&self) -> Priority {
        self.behavior.priority()
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        for d in self.draw.iter() {
            ctx.eeg.draw(d.clone());
        }

        ctx.eeg
            .draw(Drawable::print(self.behavior.blurb(), color::YELLOW));

        self.behavior.execute_old(ctx)
    }
}
