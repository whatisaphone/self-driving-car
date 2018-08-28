use behavior::{Action, Behavior};
use eeg::EEG;
use rlbot;

pub struct BehaviorRunner {
    stack: Vec<Box<Behavior>>,
}

impl BehaviorRunner {
    pub fn new(root: Box<Behavior>) -> BehaviorRunner {
        BehaviorRunner { stack: vec![root] }
    }

    pub fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> rlbot::PlayerInput {
        match self.top().execute(packet, eeg) {
            Action::Yield(result) => return result,
            Action::Call(behavior) => {
                self.stack.push(behavior);
                info!("> {}", self.top().name());
                self.execute(packet, eeg)
            }
            Action::Return => {
                if self.stack.len() == 1 {
                    panic!("Can't return from root behavior");
                }
                self.stack.pop();
                info!("< {}", self.top().name());
                self.execute(packet, eeg)
            }
        }
    }

    fn top(&mut self) -> &mut Behavior {
        &mut **self.stack.last_mut().unwrap()
    }
}
