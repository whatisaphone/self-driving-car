use crate::{
    plan::ball::BallTrajectory,
    strategy::{Context, Context2, Game, Scenario},
    utils::geometry::flattener::Flattener,
};
use common::{physics, prelude::*, rl, PrettyPrint};
use derive_new::new;
use nalgebra::{Point2, Point3, Unit, UnitComplex, UnitQuaternion, Vector2, Vector3};
use std::{fmt, iter};

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

    pub fn forward_axis_2d(&self) -> Unit<Vector2<f32>> {
        physics::car_forward_axis_2d(self.rot.to_2d())
    }

    pub fn right_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_right_axis(self.rot)
    }

    pub fn right_axis_2d(&self) -> Unit<Vector2<f32>> {
        physics::car_right_axis_2d(self.rot.to_2d())
    }

    pub fn roof_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_roof_axis(self.rot)
    }

    pub fn flatten(&self, flattener: &Flattener) -> CarState2D {
        CarState2D {
            loc: *flattener * self.loc,
            rot: *flattener * self.rot,
            vel: *flattener * self.vel,
            boost: self.boost,
        }
    }
}

impl<'a> From<&'a rlbot::ffi::PlayerInfo> for CarState {
    fn from(info: &'a rlbot::ffi::PlayerInfo) -> Self {
        Self {
            loc: info.Physics.loc(),
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
    pub fn forward_axis(&self) -> Unit<Vector2<f32>> {
        physics::car_forward_axis_2d(self.rot)
    }

    pub fn right_axis(&self) -> Unit<Vector2<f32>> {
        physics::car_right_axis_2d(self.rot)
    }

    pub fn to_3d(&self) -> CarState {
        assert!(!self.loc.x.is_nan());
        assert!(!self.vel.x.is_nan());
        CarState {
            loc: self.loc.to_3d(rl::OCTANE_NEUTRAL_Z),
            rot: self.rot.around_z_axis(),
            vel: self.vel.to_3d(0.0),
            boost: self.boost,
        }
    }
}

pub trait RoutePlanner: RoutePlannerCloneBox + Send {
    fn name(&self) -> &'static str;

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError>;
}

pub trait RoutePlannerCloneBox {
    fn clone_box(&self) -> Box<dyn RoutePlanner>;
}

impl<T> RoutePlannerCloneBox for T
where
    T: RoutePlanner + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn RoutePlanner> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn RoutePlanner> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

pub struct PlanningContext<'a: 's, 's> {
    pub game: &'s Game<'a>,
    pub start: CarState,
    pub ball_prediction: &'s BallTrajectory,
}

impl<'a: 's, 's> PlanningContext<'a, 's> {
    pub fn from_context(ctx: &Context2<'a, 's>) -> PlanningContext<'a, 's> {
        PlanningContext {
            game: &ctx.game,
            start: ctx.me().into(),
            ball_prediction: ctx.scenario.ball_prediction(),
        }
    }

    pub fn plan(
        planner: &dyn RoutePlanner,
        ctx: &mut Context<'_>,
    ) -> Result<(RoutePlan, Vec<String>), ProvisionalExpandError<'a>> {
        let (ctx, _eeg) = ctx.split();
        Self::plan2(planner, &ctx)
    }

    pub fn plan2(
        planner: &dyn RoutePlanner,
        ctx: &Context2<'_, '_>,
    ) -> Result<(RoutePlan, Vec<String>), ProvisionalExpandError<'a>> {
        let context = PlanningContext::from_context(ctx);
        Self::plan_2(planner, &context)
    }

    pub fn plan_2(
        planner: &dyn RoutePlanner,
        context: &PlanningContext<'_, '_>,
    ) -> Result<(RoutePlan, Vec<String>), ProvisionalExpandError<'a>> {
        let mut log = Vec::new();
        let mut dump = PlanningDump { log: &mut log };
        match planner.plan(context, &mut dump) {
            Ok(plan) => Ok((plan, log)),
            Err(error) => Err(ProvisionalExpandError {
                planner_name: planner.name(),
                error,
                log,
            }),
        }
    }
}

pub struct PlanningDump<'a> {
    pub log: &'a mut Vec<String>,
}

impl<'a> PlanningDump<'a> {
    fn log_line(&mut self, message: impl Into<String>) {
        self.log.push(message.into());
    }

    pub fn log(&mut self, planner: &dyn RoutePlanner, message: impl AsRef<str>) {
        self.log_line(format!("[{}] {}", planner.name(), message.as_ref()));
    }

    pub fn log_pretty(&mut self, planner: &dyn RoutePlanner, name: &str, value: impl PrettyPrint) {
        self.log_line(format!(
            "[{}] {} = {}",
            planner.name(),
            name,
            value.pretty(),
        ));
    }

    pub fn log_start(&mut self, planner: &dyn RoutePlanner, state: &CarState) {
        self.log_pretty(planner, "start loc", state.loc);
        self.log_pretty(planner, "start rot", state.rot);
        self.log_pretty(planner, "start vel", state.vel);
    }

    pub fn log_plan(&mut self, planner: &dyn RoutePlanner, plan: &RoutePlan) {
        let name = plan.segment.name();
        let end = plan.segment.end();
        let duration = plan.segment.duration();
        self.log(
            planner,
            format!("[{}] end loc = {}", name, end.loc.pretty()),
        );
        self.log(
            planner,
            format!("[{}] end rot = {}", name, end.rot.pretty()),
        );
        self.log(
            planner,
            format!("[{}] end vel = {}", name, end.vel.pretty()),
        );
        self.log(
            planner,
            format!("[{}] duration = {:.2} sec", name, duration),
        );
    }
}

pub struct ProvisionalPlanExpansionTail {
    items: Vec<Box<dyn SegmentPlan>>,
}

#[derive(new)]
pub struct ProvisionalPlanExpansion<'a> {
    head: &'a dyn SegmentPlan,
    tail: &'a ProvisionalPlanExpansionTail,
}

impl<'a> ProvisionalPlanExpansion<'a> {
    pub fn iter(&'a self) -> impl Iterator<Item = &'a (dyn SegmentPlan + 'a)> {
        iter::once(self.head).chain(self.tail.items.iter().map(|s| &**s))
    }

    pub fn duration(&self) -> f32 {
        self.iter().map(|sp| sp.duration()).sum()
    }
}

pub enum RoutePlanError {
    MustBeOnFlatGround,
    MustNotBeSkidding { recover_target_loc: Point2<f32> },
    UnknownIntercept,
    MustBeFacingTarget,
    MovingTooFast,
    TurningRadiusTooTight,
    CannotOperateWall,
    OtherError(&'static str),
}

impl fmt::Debug for RoutePlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RoutePlanError::MustBeOnFlatGround => f.write_str(stringify!(MustBeOnFlatGround)),
            RoutePlanError::MustNotBeSkidding { .. } => f.write_str(stringify!(MustNotBeSkidding)),
            RoutePlanError::UnknownIntercept => f.write_str(stringify!(UnknownIntercept)),
            RoutePlanError::MustBeFacingTarget => f.write_str(stringify!(MustBeFacingTarget)),
            RoutePlanError::MovingTooFast => f.write_str(stringify!(MovingTooFast)),
            RoutePlanError::TurningRadiusTooTight => f.write_str(stringify!(TurningRadiusTooTight)),
            RoutePlanError::CannotOperateWall => f.write_str(stringify!(CannotOperateWall)),
            RoutePlanError::OtherError(msg) => write!(f, "{}({:?})", stringify!(OtherError), msg),
        }
    }
}

#[derive(Clone)]
pub struct RoutePlan {
    pub segment: Box<dyn SegmentPlan>,
    pub next: Option<Box<dyn RoutePlanner>>,
}

impl RoutePlan {
    pub fn provisional_expand(
        &self,
        scenario: &Scenario<'_>,
    ) -> Result<ProvisionalPlanExpansionTail, ProvisionalExpandError<'_>> {
        self.provisional_expand_2(scenario.game, scenario.ball_prediction())
    }

    pub fn provisional_expand_2(
        &self,
        game: &Game<'_>,
        ball_prediction: &BallTrajectory,
    ) -> Result<ProvisionalPlanExpansionTail, ProvisionalExpandError<'_>> {
        let mut tail = Vec::new();
        if let Some(ref planner) = self.next {
            let context = PlanningContext {
                game,
                start: self.segment.end(),
                ball_prediction,
            };
            let mut log = Vec::new();
            let mut dump = PlanningDump { log: &mut log };
            match Self::expand_round(&**planner, &context, &mut dump, |s| tail.push(s)) {
                Ok(()) => {}
                Err((planner_name, error)) => {
                    return Err(ProvisionalExpandError {
                        planner_name,
                        error,
                        log,
                    });
                }
            }
        }
        Ok(ProvisionalPlanExpansionTail { items: tail })
    }

    fn expand_round(
        planner: &dyn RoutePlanner,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
        mut sink: impl FnMut(Box<dyn SegmentPlan>),
    ) -> Result<(), (&'static str, RoutePlanError)> {
        dump.log.push(format!("-{}----------", planner.name()));
        let plan = planner.plan(ctx, dump).map_err(|e| (planner.name(), e))?;
        dump.log_plan(planner, &plan);

        let state = plan.segment.end();
        let duration = plan.segment.duration();
        sink(plan.segment);

        match plan.next {
            Some(planner) => {
                let ctx = PlanningContext {
                    game: ctx.game,
                    start: state,
                    ball_prediction: &ctx.ball_prediction.hacky_expensive_slice(duration),
                };
                Self::expand_round(&*planner, &ctx, dump, sink)
            }
            None => Ok(()),
        }
    }
}

pub struct ProvisionalExpandError<'a> {
    pub planner_name: &'a str,
    pub error: RoutePlanError,
    pub log: Vec<String>,
}

pub trait SegmentPlan: SegmentPlanCloneBox + Send {
    fn name(&self) -> &str;
    fn start(&self) -> CarState;
    fn end(&self) -> CarState;
    fn duration(&self) -> f32;
    fn run(&self) -> Box<dyn SegmentRunner>;
    fn draw(&self, ctx: &mut Context<'_>);
}

pub trait SegmentPlanCloneBox {
    fn clone_box(&self) -> Box<dyn SegmentPlan>;
}

impl<T> SegmentPlanCloneBox for T
where
    T: SegmentPlan + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn SegmentPlan> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn SegmentPlan> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

pub trait SegmentRunner: Send {
    fn name(&self) -> &str;

    fn execute(&mut self, ctx: &mut Context<'_>) -> SegmentRunAction;
}

pub enum SegmentRunAction {
    Yield(rlbot::ffi::PlayerInput),
    Success,
    Failure,
}

#[cfg(test)]
mod tests {
    use crate::{
        routing::models::{CarState, PlanningContext, PlanningDump, RoutePlanner},
        strategy::Scenario,
    };
    use common::prelude::*;
    use nalgebra::{Point3, UnitComplex, Vector3};
    use std::{f32::consts::PI, mem};

    #[test]
    #[ignore(note = "Use this as needed to debug a plan.")]
    fn debug_plan() {
        // I'm very lucky that this works.
        let scenario: Scenario<'_> = unsafe { mem::zeroed() };
        let ball_prediction = unsafe { mem::zeroed() };

        let planner: &dyn RoutePlanner = unsafe { mem::uninitialized() }; // Fill this in.
        let ctx = PlanningContext {
            game: &scenario.game,
            start: CarState {
                loc: Point3::new(0.0, -4608.0, 17.01),
                rot: UnitComplex::new(PI / 2.0).around_z_axis(),
                vel: Vector3::zeros(),
                boost: 33.0,
            },
            ball_prediction: &ball_prediction,
        };
        let mut log = Vec::new();
        let mut dump = PlanningDump { log: &mut log };
        planner
            .plan(&ctx, &mut dump)
            .unwrap()
            .provisional_expand(&scenario)
            .ok()
            .unwrap();
    }
}
