#[derive(Default)]
pub struct Stopwatch {
    start: Option<f32>,
    now: Option<f32>,
}

impl Stopwatch {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn tick(&mut self, time: f32) -> f32 {
        self.start.get_or_insert(time);
        self.now = Some(time);
        self.elapsed()
    }

    pub fn elapsed(&self) -> f32 {
        self.now.unwrap() - self.start.unwrap()
    }
}
