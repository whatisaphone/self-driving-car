use crate::{
    eeg::{color, Drawable},
    strategy::{Action, Behavior, Context, Priority},
};
use itertools::Itertools;
use nameof::name_of_type;

/// Run the first `child` that does not immediately return or abort.
pub struct TryChoose {
    priority: Priority,
    choices: Vec<Box<dyn Behavior>>,
    chosen_index: Option<usize>,
    choice_names: String,
    blurb: String,
}

impl TryChoose {
    pub fn new(priority: Priority, choices: Vec<Box<dyn Behavior>>) -> Self {
        let (choice_names, blurb) = Self::blurb(choices.iter());
        Self {
            priority,
            choices,
            chosen_index: None,
            choice_names,
            blurb,
        }
    }

    fn blurb<'a>(children: impl Iterator<Item = &'a Box<dyn Behavior>>) -> (String, String) {
        let choice_names = children.map(|b| b.name()).join(", ");
        let blurb = format!("{} ({})", name_of_type!(TryChoose), choice_names);
        (choice_names, blurb)
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

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg
            .draw(Drawable::print(self.choice_names.as_str(), color::GREEN));

        if let Some(chosen_index) = self.chosen_index {
            return self.choices[chosen_index].execute_old(ctx);
        }

        // If we get here, we need to choose a child behavior. This will only happen on
        // the first frame.

        ctx.eeg.log(
            self.name(),
            format!("choosing between {}", self.choice_names),
        );

        self.exec(ctx)
    }
}

impl TryChoose {
    fn exec(&mut self, ctx: &mut Context<'_>) -> Action {
        for index in 0..self.choices.len() {
            if let Some(action) = self.try_index(ctx, index) {
                self.chosen_index = Some(index);
                return action;
            }
        }

        ctx.eeg.log(self.name(), "none suitable");
        Action::Abort
    }

    fn try_index(&mut self, ctx: &mut Context<'_>, index: usize) -> Option<Action> {
        match self.choices[index].execute_old(ctx) {
            Action::Yield(input) => {
                ctx.eeg.log(
                    self.name(),
                    format!("choosing index {}: {}", index, self.choices[index].blurb()),
                );
                return Some(Action::Yield(input));
            }
            Action::TailCall(behavior) => {
                self.choices[index] = behavior;
                ctx.eeg.log(
                    self.name(),
                    format!(
                        "diving into index {}: {}",
                        index,
                        self.choices[index].blurb(),
                    ),
                );
                return self.try_index(ctx, index);
            }
            Action::Return => None,
            Action::Abort => None,
        }
    }
}
