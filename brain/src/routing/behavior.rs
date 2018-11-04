use behavior::{Action, Behavior};
use eeg::{color, Drawable};
use routing::models::{
    ProvisionalPlanExpansion, RoutePlan, RoutePlanError, RoutePlanner, SegmentRunAction,
    SegmentRunner,
};
use strategy::Context;

pub struct FollowRoute {
    /// Option dance: This only holds a planner before the first tick.
    planner: Option<Box<RoutePlanner>>,
    current: Option<Current>,
}

struct Current {
    plan: RoutePlan,
    runner: Box<SegmentRunner>,
    provisional_expansion: ProvisionalPlanExpansion,
}

impl FollowRoute {
    pub fn new(planner: impl RoutePlanner + 'static) -> Self {
        Self {
            planner: Some(Box::new(planner)),
            current: None,
        }
    }
}

impl Behavior for FollowRoute {
    fn name(&self) -> &str {
        stringify!(FollowRoute)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if self.current.is_none() {
            let planner = &*self.planner.take().unwrap();
            if let Err(action) = self.advance(planner, ctx) {
                return action;
            }
        }

        if let Err(error) = self.draw(ctx) {
            ctx.eeg.log(format!("[FollowRoute] plan error {:?}", error));
            return Action::Abort;
        }

        self.go(ctx)
    }
}

impl FollowRoute {
    fn draw(&mut self, ctx: &mut Context) -> Result<(), RoutePlanError> {
        // This provisional expansion serves two purposes:
        // 1. Make sure each segment thinks it can complete successfully.
        // 2. Predict far enough ahead that we can draw the whole plan to the screen.
        let current = self.current.as_ref().unwrap();
        let provisional_expansion = &current.provisional_expansion;
        for segment in provisional_expansion.iter_starting_with(&*current.plan.segment) {
            segment.draw(ctx);
        }
        Ok(())
    }

    fn advance(&mut self, planner: &RoutePlanner, ctx: &mut Context) -> Result<(), Action> {
        assert!(self.current.is_none());

        let plan = match planner.plan(0.0, &ctx.me().into(), &ctx.scenario) {
            Ok(s) => s,
            Err(err) => match err.recover(ctx) {
                Some(b) => {
                    ctx.eeg
                        .log(format!("[FollowRoute] Recoverable planner error {:?}", err));
                    return Err(Action::Call(b));
                }
                None => {
                    ctx.eeg
                        .log(format!("[FollowRoute] Planner error {:?}", err));
                    return Err(Action::Abort);
                }
            },
        };
        let runner = plan.segment.run();
        let provisional_expansion =
            plan.provisional_expand(0.0, &ctx.scenario)
                .map_err(|(planner_name, error)| {
                    ctx.eeg.log(format!(
                        "[FollowRoute] Provisional expansion error from {} - {:?}",
                        planner_name, error,
                    ));
                    Action::Abort
                })?;

        self.current = Some(Current {
            plan,
            runner,
            provisional_expansion,
        });
        Ok(())
    }

    fn go(&mut self, ctx: &mut Context) -> Action {
        let action = {
            let current = self.current.as_mut().unwrap();
            ctx.eeg
                .draw(Drawable::print(current.plan.segment.name(), color::YELLOW));
            current.runner.execute(ctx)
        };

        let success = match action {
            SegmentRunAction::Yield(i) => return Action::Yield(i),
            SegmentRunAction::Success => true,
            SegmentRunAction::Failure => false,
        };

        if !success {
            ctx.eeg.log("[FollowRoute] Segment failure; aborting");
            return Action::Abort;
        }

        let current = self.current.take().unwrap();
        let next = some_or_else!(current.plan.next, {
            return Action::Return;
        });
        ctx.eeg.log("[FollowRoute] Next segment");
        if let Err(action) = self.advance(&*next, ctx) {
            return action;
        }
        self.go(ctx)
    }
}
