use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;
use strategy::{strategy, Context};

pub struct Runner2 {
    current: Option<Box<Behavior>>,
}

impl Runner2 {
    pub fn new() -> Self {
        Self { current: None }
    }
}

impl Behavior for Runner2 {
    fn name(&self) -> &'static str {
        stringify!(Runner2)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let mut ctx = Context::new(packet, eeg);
        self.exec(0, &mut ctx)
    }
}

impl Runner2 {
    fn exec(&mut self, depth: u32, ctx: &mut Context) -> Action {
        if depth > 5 {
            ctx.eeg.log("infinite loop?");
            return Action::Yield(Default::default());
        }

        let action = {
            let behavior = self.choose_behavior(ctx);
            ctx.eeg
                .draw(Drawable::print(behavior.name(), color::YELLOW));
            behavior.execute2(ctx)
        };

        match action {
            Action::Yield(i) => Action::Yield(i),
            Action::Call(b) => {
                ctx.eeg.log(format!("> {}", b.name()));
                self.current = Some(b);
                self.exec(depth + 1, ctx)
            }
            Action::Return => {
                ctx.eeg
                    .log(format!("< {}", self.current.as_ref().unwrap().name()));
                self.current = None;
                self.exec(depth + 1, ctx)
            }
        }
    }

    fn choose_behavior(&mut self, ctx: &mut Context) -> &mut Behavior {
        if self.current.is_none() {
            self.current = Some(strategy::baseline(ctx));
            ctx.eeg.log(format!(
                "baseline: {}",
                self.current.as_ref().unwrap().name()
            ));
        }

        if let Some(b) = strategy::override_(ctx, &**self.current.as_ref().unwrap()) {
            self.current = Some(b);
            ctx.eeg.log(format!(
                "override: {}",
                self.current.as_ref().unwrap().name()
            ));
        }

        &mut **self.current.as_mut().unwrap()
    }
}
