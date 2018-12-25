use crate::{
    eeg::{color, Drawable},
    strategy::{strategy::Strategy, Action, Behavior, Context},
};
use nameof::name_of_type;
use rlbot;

pub const BASELINE: &str = "baseline:";

pub struct Runner {
    strategy: Box<Strategy>,
    current: Option<Box<Behavior>>,
}

impl Runner {
    pub fn new(strategy: impl Strategy + 'static) -> Self {
        Self {
            strategy: Box::new(strategy),
            current: None,
        }
    }

    #[cfg(test)]
    pub fn soccar() -> Self {
        Self {
            strategy: Box::new(crate::strategy::Soccar::new()),
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

    pub fn execute(&mut self, ctx: &mut Context) -> rlbot::ffi::PlayerInput {
        self.exec(0, ctx)
    }
}

impl Behavior for Runner {
    fn name(&self) -> &str {
        name_of_type!(Runner)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        Action::Yield(self.exec(0, ctx))
    }
}

impl Runner {
    fn exec(&mut self, depth: u32, ctx: &mut Context) -> rlbot::ffi::PlayerInput {
        if depth > 5 {
            ctx.eeg.log("infinite loop?");
            return Default::default();
        }

        let behavior = self.choose_behavior(ctx);
        ctx.eeg
            .draw(Drawable::print(behavior.name(), color::YELLOW));
        let action = behavior.execute2(ctx);

        match action {
            Action::Yield(i) => i,
            Action::Call(b) => {
                ctx.eeg.log(format!("> {}", b.name()));
                self.current = Some(b);
                self.exec(depth + 1, ctx)
            }
            Action::Return | Action::Abort => {
                ctx.eeg
                    .log(format!("< {}", self.current.as_ref().unwrap().name()));
                self.current = None;
                self.exec(depth + 1, ctx)
            }
        }
    }

    fn choose_behavior(&mut self, ctx: &mut Context) -> &mut Behavior {
        if self.current.is_none() {
            self.current = Some(self.strategy.baseline(ctx));
            ctx.eeg.log(format!(
                "{} {}",
                BASELINE,
                self.current.as_ref().unwrap().name()
            ));
        }

        if let Some(b) = self
            .strategy
            .interrupt(ctx, &**self.current.as_ref().unwrap())
        {
            self.current = Some(b);
            ctx.eeg.log(format!(
                "override: {}",
                self.current.as_ref().unwrap().name()
            ));
        }

        &mut **self.current.as_mut().unwrap()
    }
}
