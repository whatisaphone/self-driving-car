use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct BehaviorRunner {
    stack: Vec<Box<Behavior>>,
}

impl BehaviorRunner {
    pub fn new(root: Box<Behavior>) -> BehaviorRunner {
        BehaviorRunner { stack: vec![root] }
    }

    pub fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput {
        match self.stack.last_mut().unwrap().execute(packet, eeg) {
            Action::Yield(result) => return result,
            Action::Call(behavior) => {
                self.stack.push(behavior);
                self.execute(packet, eeg)
            }
            Action::Return => {
                if self.stack.len() == 1 {
                    panic!("Can't return from root behavior");
                }
                self.stack.pop();
                self.execute(packet, eeg)
            }
        }
    }
}
