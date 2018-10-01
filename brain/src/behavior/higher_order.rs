use behavior::{Action, Behavior, Priority};
use eeg::{color, Drawable};
use itertools::Itertools;
use std::{collections::VecDeque, iter};
use strategy::Context;

/// Run `child` until it returns, then do nothing forever.

#[allow(dead_code)]
pub struct Fuse {
    child: Option<Box<Behavior>>,
}

impl Fuse {
    #[allow(dead_code)]
    pub fn new(child: Box<Behavior>) -> Self {
        Self { child: Some(child) }
    }
}

impl Behavior for Fuse {
    fn name(&self) -> &str {
        stringify!(Fuse)
    }

    fn execute2(&mut self, _ctx: &mut Context) -> Action {
        // `take()` leaves a None behind, so this can only match `Some` once.
        match self.child.take() {
            Some(b) => Action::Call(b),
            None => Action::Yield(Default::default()),
        }
    }
}

/// Do `behavior` forever
#[allow(dead_code)]
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
    #[allow(dead_code)]
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

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg
            .draw(Drawable::print(self.current.name(), color::YELLOW));
        match self.current.execute2(ctx) {
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

/// Execute `child` for at most `limit` seconds, then return.
pub struct TimeLimit {
    limit: f32,
    child: Box<Behavior>,
    start: Option<f32>,
}

impl TimeLimit {
    pub fn new(limit: f32, child: impl Behavior + 'static) -> Self {
        Self {
            limit,
            child: Box::new(child),
            start: None,
        }
    }
}

impl Behavior for TimeLimit {
    fn name(&self) -> &str {
        stringify!(TimeLimit)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let now = ctx.packet.GameInfo.TimeSeconds;
        let start = *self.start.get_or_insert(now);
        let elapsed = now - start;
        if elapsed >= self.limit {
            Action::Return
        } else {
            self.child.execute2(ctx)
        }
    }
}

/// Run `children` in sequence.
pub struct Chain {
    priority: Priority,
    children: VecDeque<Box<Behavior>>,
    name: String,
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
        iter::once(stringify!(Chain))
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

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        ctx.eeg.draw(Drawable::print(
            self.children
                .iter()
                .map(|b| b.name())
                .collect::<Vec<_>>()
                .join(", "),
            color::GREEN,
        ));

        let action = {
            let mut front = match self.children.front_mut() {
                None => return Action::Return,
                Some(b) => b,
            };
            ctx.eeg.draw(Drawable::print(front.name(), color::YELLOW));
            front.execute2(ctx)
        };

        match action {
            Action::Yield(x) => Action::Yield(x),
            Action::Call(b) => {
                ctx.eeg
                    .log(format!("[Chain] replacing head with {}", b.name()));
                self.children[0] = b;
                self.name = Self::name(self.children.iter());
                self.execute2(ctx)
            }
            Action::Return => {
                ctx.eeg.log("[Chain] advancing");
                self.children.pop_front();
                self.name = Self::name(self.children.iter());
                self.execute2(ctx)
            }
            Action::Abort => {
                ctx.eeg.log("[Chain] aborting");
                Action::Abort
            }
        }
    }
}
