use behavior::{Action, Behavior, Priority};
use eeg::{color, Drawable, EEG};
use rlbot;
use std::collections::VecDeque;
use strategy::Context;

/// Run `child` until it returns, then do nothing forever.

#[allow(dead_code)]
pub struct Fuse {
    child: Option<Box<Behavior>>,
}

impl Fuse {
    #[allow(dead_code)]
    pub fn new(child: Box<Behavior>) -> Fuse {
        Fuse { child: Some(child) }
    }
}

impl Behavior for Fuse {
    fn name(&self) -> &'static str {
        stringify!(Fuse)
    }

    fn execute(&mut self, _packet: &rlbot::LiveDataPacket, _eeg: &mut EEG) -> Action {
        // `take()` leaves a None behind, so this can only match `Some` once.
        match self.child.take() {
            Some(b) => Action::Call(b),
            None => Action::Yield(Default::default()),
        }
    }
}

/// Do nothing for `time` seconds, then run `child`.
#[allow(dead_code)]
pub struct Delay {
    time: f32,
    child: Option<Box<Behavior>>,
}

impl Delay {
    #[allow(dead_code)]
    pub fn new(time: f32, child: Box<Behavior>) -> Delay {
        Delay {
            time,
            child: Some(child),
        }
    }
}

impl Behavior for Delay {
    fn name(&self) -> &'static str {
        stringify!(Delay)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, _eeg: &mut EEG) -> Action {
        if packet.GameInfo.TimeSeconds < self.time {
            Action::Yield(Default::default())
        } else {
            // `take()` leaves a None behind, so this can only match `Some` once.
            match self.child.take() {
                Some(b) => Action::Call(b),
                None => Action::Yield(Default::default()),
            }
        }
    }
}

/// Run `children` in sequence.
pub struct Chain {
    priority: Priority,
    children: VecDeque<Box<Behavior>>,
}

impl Chain {
    pub fn new(priority: Priority, children: Vec<Box<Behavior>>) -> Self {
        Self {
            priority,
            children: children.into_iter().collect(),
        }
    }
}

impl Behavior for Chain {
    fn name(&self) -> &'static str {
        stringify!(Chain)
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
                self.execute2(ctx)
            }
            Action::Return => {
                ctx.eeg.log("[Chain] advancing");
                self.children.pop_front();
                self.execute2(ctx)
            }
            Action::Abort => {
                ctx.eeg.log("[Chain] aborting");
                Action::Abort
            }
        }
    }
}
