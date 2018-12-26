use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};
use itertools::Itertools;
use nameof::name_of_type;
use std::iter;

/// Run the first `child` that does not immediately return or abort.
pub struct TryChoose {
    priority: Priority,
    choices: Vec<Box<Behavior>>,
    chosen_index: Option<usize>,
    blurb: String,
}

impl TryChoose {
    pub fn new(priority: Priority, choices: Vec<Box<Behavior>>) -> Self {
        let blurb = Self::blurb(choices.iter());
        Self {
            priority,
            choices,
            chosen_index: None,
            blurb,
        }
    }

    fn blurb<'a>(children: impl Iterator<Item = &'a Box<Behavior>>) -> String {
        iter::once(name_of_type!(TryChoose))
            .chain(iter::once(" ("))
            .chain(children.map(|b| b.name()).intersperse(", "))
            .chain(iter::once(")"))
            .join("")
    }
}

impl Behavior for TryChoose {
    fn name(&self) -> &str {
        name_of_type!(TryChoose)
    }

    fn blurb(&self) -> &str {
        &self.blurb
    }

    fn priority(&self) -> Priority {
        self.priority
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg.draw(Drawable::print(
            self.choices
                .iter()
                .map(|b| b.name())
                .collect::<Vec<_>>()
                .join(", "),
            color::GREEN,
        ));

        if let Some(chosen_index) = self.chosen_index {
            return self.choices[chosen_index].execute(ctx);
        }

        // We need to choose a child behavior. This will happen on the first frame.

        for (index, behavior) in self.choices.iter_mut().enumerate() {
            match behavior.execute(ctx) {
                Action::Abort => continue,
                Action::Return => continue,
                action => {
                    self.chosen_index = Some(index);
                    ctx.eeg.log(self.name(), format!("chose index {}", index));
                    return action;
                }
            }
        }

        ctx.eeg.log(self.name(), "none suitable");
        Action::Abort
    }
}
