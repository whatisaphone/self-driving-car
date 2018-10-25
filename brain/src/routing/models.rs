use common::{ext::ExtendPhysics, physics};
use nalgebra::{Point2, Point3, Unit, UnitComplex, UnitQuaternion, Vector2, Vector3};
use rlbot;
use simulate::rl;
use strategy::Context;
use utils::geometry::{ExtendPoint2, ExtendUnitComplex, ExtendVector2};

#[derive(Clone)]
pub struct CarState {
    pub loc: Point3<f32>,
    pub rot: UnitQuaternion<f32>,
    pub vel: Vector3<f32>,
    pub boost: f32,
}

impl CarState {
    pub fn forward_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_forward_axis(self.rot)
    }

    pub fn right_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_right_axis(self.rot)
    }
}

impl<'a> From<&'a rlbot::ffi::PlayerInfo> for CarState {
    fn from(info: &'a rlbot::ffi::PlayerInfo) -> Self {
        Self {
            loc: info.Physics.locp(),
            rot: info.Physics.quat(),
            vel: info.Physics.vel(),
            boost: info.Boost as f32,
        }
    }
}

pub struct CarState2D {
    pub loc: Point2<f32>,
    pub rot: UnitComplex<f32>,
    pub vel: Vector2<f32>,
    pub boost: f32,
}

impl CarState2D {
    pub fn to_3d(&self) -> CarState {
        CarState {
            loc: self.loc.to_3d(rl::OCTANE_NEUTRAL_Z),
            rot: self.rot.around_z_axis(),
            vel: self.vel.to_3d(0.0),
            boost: self.boost,
        }
    }
}

pub trait SegmentPlan: Send {
    fn start(&self) -> CarState;
    fn end(&self) -> CarState;
    fn duration(&self) -> f32;
    fn truncate_to_duration(&self, duration: f32) -> Box<SegmentPlan>;
    fn run(&self) -> Box<SegmentRunner>;
    fn draw(&self, ctx: &mut Context);
}

pub trait SegmentRunner: Send {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction;
}

pub enum SegmentRunAction {
    Yield(rlbot::ffi::PlayerInput),
    Success,
    Failure,
}

pub struct RoutePlan {
    pub(in routing) segments: Vec<Box<SegmentPlan>>,
}

impl RoutePlan {
    pub(in routing) fn new(segments: Vec<Box<SegmentPlan>>) -> Self {
        Self { segments }
    }

    pub fn end(&self) -> CarState {
        self.segments.last().unwrap().end()
    }

    pub fn duration(&self) -> f32 {
        self.segments.iter().map(|s| s.duration()).sum()
    }

    pub fn draw(&self, ctx: &mut Context) {
        for segment in self.segments.iter() {
            segment.draw(ctx);
        }
    }

    /// Modify this plan so it finishes in a shorter time.
    ///
    /// The shortened plan will of course not accomplish everything the
    /// original plan does, because there is no longer enough time.
    pub fn truncate_to_duration(self, target_duration: f32) -> Option<Self> {
        let cur_duration = self.duration();
        if target_duration <= 0.0 {
            return None;
        } else if target_duration >= cur_duration {
            return Some(self);
        }

        let split = Self::calc_split(self.segments.iter().map(|s| s.duration()), target_duration);
        let (index, cut_duration) = some_or_else!(split, {
            return None;
        });

        let cut = self.segments[index].truncate_to_duration(cut_duration);

        let mut segments = self.segments;
        segments.truncate(index);
        segments.push(cut);
        Some(Self { segments })
    }

    fn calc_split(xs: impl Iterator<Item = f32>, target: f32) -> Option<(usize, f32)> {
        let mut total = 0.0;
        for (i, x) in xs.enumerate() {
            if total + x >= target {
                return Some((i, target - total));
            }
            total += x;
        }
        return None;
    }
}
