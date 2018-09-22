use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;
use strategy::Context;

pub struct BehaviorRunner {
    stack: Vec<Box<Behavior>>,
}

pub const PUSHED: &str = ">";
pub const POPPED: &str = "<";
pub const UNWOUND: &str = "<<";

impl BehaviorRunner {
    pub fn new(root: Box<Behavior>) -> BehaviorRunner {
        BehaviorRunner { stack: vec![root] }
    }

    pub fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput {
        let mut ctx = Context::new(packet, eeg);
        self.recurse(0, &mut ctx)
    }

    pub fn recurse(&mut self, depth: i32, ctx: &mut Context) -> rlbot::PlayerInput {
        if depth >= 5 {
            ctx.eeg.log("Infinite loop?");
            return Default::default();
        }

        let capture = 'capture: loop {
            for (bi, behavior) in self.stack.iter_mut().enumerate() {
                ctx.eeg
                    .draw(Drawable::print(behavior.name(), color::YELLOW));
                if let Some(action) = behavior.capture(ctx.packet, ctx.eeg) {
                    break 'capture Some((bi, action));
                }
            }
            break None;
        };

        let action = match capture {
            Some((bi, action)) => {
                self.stack.truncate(bi + 1);
                ctx.eeg.log(format!("{} {}", UNWOUND, self.top().name()));
                action
            }
            None => self.top().execute2(ctx),
        };

        match action {
            Action::Yield(result) => return result,
            Action::Call(behavior) => {
                self.stack.push(behavior);
                ctx.eeg.log(format!("{} {}", PUSHED, self.top().name()));
                self.recurse(depth + 1, ctx)
            }
            Action::Return => {
                if self.stack.len() == 1 {
                    panic!("Can't return from root behavior");
                }
                self.stack.pop();
                ctx.eeg.log(format!("{} {}", POPPED, self.top().name()));
                self.recurse(depth + 1, ctx)
            }
        }
    }

    fn top(&mut self) -> &mut Behavior {
        &mut **self.stack.last_mut().unwrap()
    }
}
