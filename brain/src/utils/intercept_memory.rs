use crate::eeg::EEG;
use nalgebra::Point3;
use nameof::name_of_type;

#[derive(Default)]
pub struct InterceptMemory {
    current: Option<Point3<f32>>,
    watch: Option<(f32, Point3<f32>)>,
}

impl InterceptMemory {
    const LOC_THRESHOLD: f32 = 100.0;
    const TIME_THRESHOLD: f32 = 0.1;

    #[cfg(test)]
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, time: f32, loc: Point3<f32>, eeg: &mut EEG) -> InterceptMemoryResult {
        match self.calc(time, loc) {
            InterceptMemoryAction::Trust => {
                self.current = Some(loc);
                self.watch = None;
                InterceptMemoryResult::Stable(loc)
            }
            InterceptMemoryAction::Watch => {
                eeg.log(name_of_type!(InterceptMemory), "watching for change");
                self.watch = Some((time, loc));
                InterceptMemoryResult::Stable(self.current.unwrap())
            }
            InterceptMemoryAction::Wait => InterceptMemoryResult::Stable(self.current.unwrap()),
            InterceptMemoryAction::Replace => {
                eeg.log(
                    name_of_type!(InterceptMemory),
                    "replacing previous intercept",
                );
                self.current = Some(loc);
                self.watch = None;
                InterceptMemoryResult::Unstable(loc)
            }
        }
    }

    fn calc(&self, time: f32, loc: Point3<f32>) -> InterceptMemoryAction {
        let current = some_or_else!(self.current, {
            return InterceptMemoryAction::Trust;
        });
        if (loc - current).norm() < Self::LOC_THRESHOLD {
            return InterceptMemoryAction::Trust;
        }
        let (watch_time, watch_loc) = some_or_else!(self.watch, {
            return InterceptMemoryAction::Watch;
        });
        if (loc - watch_loc).norm() >= Self::LOC_THRESHOLD {
            return InterceptMemoryAction::Watch;
        }
        if time - watch_time < Self::TIME_THRESHOLD {
            return InterceptMemoryAction::Wait;
        }
        InterceptMemoryAction::Replace
    }
}

pub enum InterceptMemoryResult {
    Stable(Point3<f32>),
    Unstable(Point3<f32>),
}

enum InterceptMemoryAction {
    Trust,
    Watch,
    Wait,
    Replace,
}
