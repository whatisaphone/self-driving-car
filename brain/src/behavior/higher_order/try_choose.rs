use crate::{
    behavior::{Action, Behavior, Priority},
    eeg::{color, Drawable},
    strategy::Context,
};
use itertools::Itertools;
use nameof::name_of_type;
use std::iter;

/// Run the first `child` that does not immediately return or abort.
pub struct TryChoose {
    priority: Priority,
    choices: Vec<Box<Behavior>>,
    chosen_index: Option<usize>,
    name: String,
}

impl TryChoose {
    pub fn new(priority: Priority, choices: Vec<Box<Behavior>>) -> Self {
        let name = Self::name(choices.iter());
        Self {
            priority,
            choices,
            chosen_index: None,
            name,
        }
    }

    fn name<'a>(children: impl Iterator<Item = &'a Box<Behavior>>) -> String {
        iter::once(name_of_type!(TryChoose))
            .chain(iter::once(" ("))
            .chain(children.map(|b| b.name()).intersperse(", "))
            .chain(iter::once(")"))
            .join("")
    }
}

impl Behavior for TryChoose {
    fn name(&self) -> &str {
        &self.name
    }

    fn priority(&self) -> Priority {
        self.priority
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg.draw(Drawable::print(
            self.choices
                .iter()
                .map(|b| b.name())
                .collect::<Vec<_>>()
                .join(", "),
            color::GREEN,
        ));

        if let Some(chosen_index) = self.chosen_index {
            return self.choices[chosen_index].execute2(ctx);
        }

        // We need to choose a child behavior. This will happen on the first frame.

        for (index, behavior) in self.choices.iter_mut().enumerate() {
            match behavior.execute2(ctx) {
                Action::Abort => continue,
                Action::Return => continue,
                action @ _ => {
                    self.chosen_index = Some(index);
                    ctx.eeg.log(format!("[TryChoose] Chose index {}", index));
                    return action;
                }
            }
        }

        ctx.eeg.log("[TryChoose] None suitable");
        return Action::Abort;
    }
}
