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
        let capture = 'capture: loop {
            for (bi, behavior) in self.stack.iter_mut().enumerate() {
                if let Some(action) = behavior.capture(packet, eeg) {
                    break 'capture Some((bi, action));
                }
            }
            break None;
        };

        let action = match capture {
            Some((bi, action)) => {
                self.stack.truncate(bi + 1);
                warn!("<< {}", self.top().name());
                action
            }
            None => self.top().execute(packet, eeg),
        };

        match action {
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
