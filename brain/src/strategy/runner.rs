use crate::{
    eeg::{color, Drawable},
    strategy::{strategy::Strategy, Action, Behavior, Context},
};
use nameof::name_of_type;

pub struct Runner {
    strategy: Box<dyn Strategy>,
    current: Option<Box<dyn Behavior>>,
}

impl Runner {
    pub fn new(strategy: impl Strategy + 'static) -> Self {
        Self {
            strategy: Box::new(strategy),
            current: None,
        }
    }

    #[cfg(test)]
    pub fn with_current(current: impl Behavior + 'static) -> Self {
        Self {
            strategy: Box::new(crate::strategy::null::NullStrategy::new()),
            current: Some(Box::new(current)),
        }
    }

    pub fn execute_old(&mut self, ctx: &mut Context<'_>) -> common::halfway_house::PlayerInput {
        self.exec(0, ctx)
    }
}

impl Behavior for Runner {
    fn name(&self) -> &str {
        name_of_type!(Runner)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        Action::Yield(self.exec(0, ctx))
    }
}

impl Runner {
    fn exec(&mut self, depth: u32, ctx: &mut Context<'_>) -> common::halfway_house::PlayerInput {
        if depth > 5 {
            ctx.eeg.log(self.name(), "infinite loop?");
            return Default::default();
        }

        let behavior = self.choose_behavior(ctx);
        ctx.eeg
            .draw(Drawable::print(behavior.blurb(), color::YELLOW));

        match behavior.execute_old(ctx) {
            Action::Yield(i) => i,
            Action::TailCall(b) => {
                ctx.eeg.log(self.name(), format!("> {}", b.name()));
                self.current = Some(b);
                self.exec(depth + 1, ctx)
            }
            Action::Return | Action::Abort => {
                ctx.eeg.log(
                    self.name(),
                    format!("< {}", self.current.as_ref().unwrap().name()),
                );
                self.current = None;
                self.exec(depth + 1, ctx)
            }
        }
    }

    fn choose_behavior(&mut self, ctx: &mut Context<'_>) -> &mut dyn Behavior {
        if self.current.is_none() {
            self.current = Some(self.strategy.baseline(ctx));
            ctx.eeg.log(
                self.name(),
                format!("baseline: {}", self.current.as_ref().unwrap().name()),
            );
        }

        if let Some(b) = self
            .strategy
            .interrupt(ctx, &**self.current.as_ref().unwrap())
        {
            self.current = Some(b);
            ctx.eeg.log(
                self.name(),
                format!("override: {}", self.current.as_ref().unwrap().name()),
            );
        }

        &mut **self.current.as_mut().unwrap()
    }
}
