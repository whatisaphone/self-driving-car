use crate::{
    behavior::{Action, Behavior, Priority},
    eeg::{color, Drawable},
    strategy::Context,
};
use itertools::Itertools;
use std::{collections::VecDeque, iter};

/// Run `child` until it returns, then do nothing forever.

pub struct Fuse {
    child: Option<Box<Behavior>>,
}

impl Fuse {
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

/// Run `behavior` forever
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
            Action::Abort
        } else {
            self.child.execute2(ctx)
        }
    }
}

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
        use crate::behavior::Chain;
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

        let front = match self.children.front_mut() {
            None => return Action::Return,
            Some(b) => b,
        };
        ctx.eeg.draw(Drawable::print(front.name(), color::YELLOW));

        match front.execute2(ctx) {
            Action::Yield(x) => Action::Yield(x),
            Action::Call(b) => {
                self.children[0] = b;
                self.name = Self::name(self.children.iter());
                ctx.eeg
                    .log(format!("[Chain] child Call; becoming {}", self.name));
                self.execute2(ctx)
            }
            Action::Return => {
                self.children.pop_front();
                self.name = Self::name(self.children.iter());
                ctx.eeg
                    .log(format!("[Chain] child Return; becoming {}", self.name));
                self.execute2(ctx)
            }
            Action::Abort => {
                ctx.eeg.log("[Chain] child Abort");
                Action::Abort
            }
        }
    }
}

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
        iter::once(stringify!(TryChoose))
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

/// Run `child` while `predicate` holds true.
pub struct While<P, B>
where
    P: Predicate,
    B: Behavior,
{
    predicate: P,
    child: B,
}

pub trait Predicate: Send {
    fn name(&self) -> &str;
    fn evaluate(&mut self, ctx: &mut Context) -> bool;
}

impl<P, B> While<P, B>
where
    P: Predicate,
    B: Behavior,
{
    pub fn new(predicate: P, child: B) -> Self {
        Self { predicate, child }
    }
}

impl<P, B> Behavior for While<P, B>
where
    P: Predicate,
    B: Behavior,
{
    fn name(&self) -> &str {
        stringify!(While)
    }

    fn priority(&self) -> Priority {
        self.child.priority()
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if !self.predicate.evaluate(ctx) {
            ctx.eeg.log("[While] Terminating");
            return Action::Abort;
        }

        ctx.eeg
            .draw(Drawable::print(self.child.name(), color::YELLOW));

        self.child.execute2(ctx)
    }
}

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

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        for d in self.draw.iter() {
            ctx.eeg.draw(d.clone());
        }

        ctx.eeg
            .draw(Drawable::print(self.behavior.name(), color::YELLOW));

        self.behavior.execute2(ctx)
    }
}
