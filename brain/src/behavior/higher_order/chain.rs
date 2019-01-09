use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};
use itertools::Itertools;
use nameof::name_of_type;
use std::{collections::VecDeque, iter};

/// Run `children` in sequence.
pub struct Chain {
    priority: Priority,
    children: VecDeque<Box<dyn Behavior>>,
    /// Cache the full name of the Behavior, including names of `children`. This
    /// must be kept up to date whenever `children` is modified.
    blurb: String,
}

macro_rules! chain {
    ($priority:expr, [$($child:expr),+ $(,)*] $(,)*) => {{
        use crate::behavior::higher_order::Chain;
        Chain::new($priority, vec![$(Box::new($child)),+])
    }}
}

impl Chain {
    pub fn new(priority: Priority, children: Vec<Box<dyn Behavior>>) -> Self {
        Self {
            blurb: Self::blurb(children.iter()),
            priority,
            children: children.into_iter().collect(),
        }
    }

    fn blurb<'a>(children: impl Iterator<Item = &'a Box<dyn Behavior>>) -> String {
        iter::once(name_of_type!(Chain))
            .chain(iter::once(" ("))
            .chain(children.map(|b| b.name()).intersperse(", "))
            .chain(iter::once(")"))
            .join("")
    }
}

impl Behavior for Chain {
    fn name(&self) -> &str {
        name_of_type!(Chain)
    }

    fn blurb(&self) -> &str {
        &self.blurb
    }

    fn priority(&self) -> Priority {
        self.priority
    }

    fn execute(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg.draw(Drawable::print(
            self.children
                .iter()
                .map(|b| b.name())
                .collect::<Vec<_>>()
                .join(", "),
            color::GREEN,
        ));

        let front = match self.children.front_mut() {
            None => return Action::Return,
            Some(b) => b,
        };
        ctx.eeg.draw(Drawable::print(front.blurb(), color::YELLOW));

        match front.execute(ctx) {
            Action::Yield(x) => Action::Yield(x),
            Action::Call(b) => {
                self.children[0] = b;
                self.blurb = Self::blurb(self.children.iter());
                ctx.eeg
                    .log(self.name(), format!("child Call; becoming {}", self.blurb));
                self.execute(ctx)
            }
            Action::Return => {
                self.children.pop_front();
                self.blurb = Self::blurb(self.children.iter());
                ctx.eeg.log(
                    self.name(),
                    format!("child Return; becoming {}", self.blurb),
                );
                self.execute(ctx)
            }
            Action::Abort => {
                ctx.eeg.log(self.name(), "child Abort");
                Action::Abort
            }
        }
    }
}
