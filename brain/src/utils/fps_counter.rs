use std::collections::VecDeque;

pub struct FPSCounter {
    times: VecDeque<f32>,
    ready: bool,
}

impl FPSCounter {
    pub fn new() -> FPSCounter {
        FPSCounter {
            times: VecDeque::new(),
            ready: false,
        }
    }

    pub fn fps(&self) -> Option<usize> {
        if self.ready {
            Some(self.times.len())
        } else {
            None
        }
    }

    pub fn tick(&mut self, time: f32) {
        if !self.times.is_empty() && time < *self.times.back().unwrap() {
            warn!("Why is time moving backwards?");
            self.times.clear();
            self.ready = false;
        }

        self.times.push_back(time);
        while !self.times.is_empty() && *self.times.front().unwrap() < time - 1.0 {
            self.times.pop_front();
            self.ready = true;
        }
    }
}
