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
    children: VecDeque<Box<Behavior>>,
    /// Cache the full name of the Behavior, including names of `children`. This
    /// must be kept up to date whenever `children` is modified.
    name: String,
}

macro_rules! chain {
    ($priority:expr, [$($child:expr),+ $(,)*] $(,)*) => {{
        use crate::behavior::higher_order::Chain;
        Chain::new($priority, vec![$(Box::new($child)),+])
    }}
}

impl Chain {
    pub fn new(priority: Priority, children: Vec<Box<Behavior>>) -> Self {
        Self {
            name: Self::name(children.iter()),
            priority,
            children: children.into_iter().collect(),
        }
    }

    fn name<'a>(children: impl Iterator<Item = &'a Box<Behavior>>) -> String {
        iter::once(name_of_type!(Chain))
            .chain(iter::once(" ("))
            .chain(children.map(|b| b.name()).intersperse(", "))
            .chain(iter::once(")"))
            .join("")
    }
}

impl Behavior for Chain {
    fn name(&self) -> &str {
        &self.name
    }

    fn priority(&self) -> Priority {
        self.priority
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
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
        ctx.eeg.draw(Drawable::print(front.name(), color::YELLOW));

        match front.execute(ctx) {
            Action::Yield(x) => Action::Yield(x),
            Action::Call(b) => {
                self.children[0] = b;
                self.name = Self::name(self.children.iter());
                ctx.eeg.log(
                    name_of_type!(Chain),
                    format!("child Call; becoming {}", self.name),
                );
                self.execute(ctx)
            }
            Action::Return => {
                self.children.pop_front();
                self.name = Self::name(self.children.iter());
                ctx.eeg.log(
                    name_of_type!(Chain),
                    format!("child Return; becoming {}", self.name),
                );
                self.execute(ctx)
            }
            Action::Abort => {
                ctx.eeg.log(name_of_type!(Chain), "child Abort");
                Action::Abort
            }
        }
    }
}
