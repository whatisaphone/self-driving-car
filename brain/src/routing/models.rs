use common::{ext::ExtendPhysics, physics};
use nalgebra::{Point2, Point3, Unit, UnitComplex, UnitQuaternion, Vector2, Vector3};
use rlbot;
use simulate::rl;
use std::iter;
use strategy::{Context, Scenario};
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

#[derive(Clone)]
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

pub trait RoutePlanner: RoutePlannerCloneBox + Send {
    fn plan(&self, start: &CarState, scenario: &Scenario) -> Result<RouteStep, RoutePlanError>;
}

impl RouteStep {
    pub fn provisional_expand<'a>(
        &'a self,
        scenario: &Scenario,
    ) -> Result<ProvisionalPlanExpansion<'a>, RoutePlanError> {
        let mut tail = Vec::new();
        if let Some(ref planner) = self.next {
            Self::expand_round(&**planner, &self.segment.end(), scenario, |s| tail.push(s))?;
        }
        Ok(ProvisionalPlanExpansion {
            head: &*self.segment,
            tail,
        })
    }

    fn expand_round(
        planner: &RoutePlanner,
        state: &CarState,
        scenario: &Scenario,
        mut sink: impl FnMut(Box<SegmentPlan>),
    ) -> Result<(), RoutePlanError> {
        let step = planner.plan(&state, scenario)?;
        let state = step.segment.end();
        sink(step.segment);
        match step.next {
            Some(planner) => Self::expand_round(&*planner, &state, scenario, sink),
            None => Ok(()),
        }
    }
}

pub struct ProvisionalPlanExpansion<'a> {
    head: &'a SegmentPlan,
    tail: Vec<Box<SegmentPlan>>,
}

impl<'a> ProvisionalPlanExpansion<'a> {
    pub fn iter(&'a self) -> impl Iterator<Item = &'a SegmentPlan> {
        iter::once(self.head).chain(self.tail.iter().map(|s| &**s))
    }
}

pub trait RoutePlannerCloneBox {
    fn clone_box(&self) -> Box<RoutePlanner>;
}

impl<T: RoutePlanner + Clone + 'static> RoutePlannerCloneBox for T {
    fn clone_box(&self) -> Box<RoutePlanner> {
        Box::new(self.clone())
    }
}

#[derive(Debug)]
pub enum RoutePlanError {
    MustBeOnFlatGround,
    MustNotBeSkidding,
    UnknownIntercept,
    OtherError(&'static str),
}

pub struct RouteStep {
    pub segment: Box<SegmentPlan>,
    pub next: Option<Box<RoutePlanner>>,
}

pub trait SegmentPlan: Send {
    fn start(&self) -> CarState;
    fn end(&self) -> CarState;
    fn duration(&self) -> f32;
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
