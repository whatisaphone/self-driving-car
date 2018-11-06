use behavior::{Action, Behavior};
use eeg::{color, Drawable};
use routing::models::{
    PlanningContext, PlanningDump, ProvisionalPlanExpansion, RoutePlan, RoutePlanner,
    SegmentRunAction, SegmentRunner,
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

        self.draw(ctx);
        self.go(ctx)
    }
}

impl FollowRoute {
    fn draw(&mut self, ctx: &mut Context) {
        // This provisional expansion serves two purposes:
        // 1. Make sure each segment thinks it can complete successfully.
        // 2. Predict far enough ahead that we can draw the whole plan to the screen.
        let current = self.current.as_ref().unwrap();
        let provisional_expansion = &current.provisional_expansion;
        for segment in provisional_expansion.iter_starting_with(&*current.plan.segment) {
            segment.draw(ctx);
        }
    }

    fn advance(&mut self, planner: &RoutePlanner, ctx: &mut Context) -> Result<(), Action> {
        assert!(self.current.is_none());

        let mut log = Vec::new();
        let plan = {
            let mut context = PlanningContext {
                game: &ctx.game,
                start: ctx.me().into(),
                ball_prediction: ctx.scenario.ball_prediction(),
            };
            let mut dump = PlanningDump { log: &mut log };
            planner.plan(&context, &mut dump)
        };
        let plan = match plan {
            Ok(s) => s,
            Err(err) => match err.recover(ctx) {
                Some(b) => {
                    ctx.eeg
                        .log(format!("[FollowRoute] Recoverable planner error {:?}", err));
                    for line in log {
                        ctx.eeg.log(format!("[FollowRoute] {}", line));
                    }
                    return Err(Action::Call(b));
                }
                None => {
                    ctx.eeg
                        .log(format!("[FollowRoute] Planner error {:?}", err));
                    for line in log {
                        ctx.eeg.log(format!("[FollowRoute] {}", line));
                    }
                    return Err(Action::Abort);
                }
            },
        };
        let runner = plan.segment.run();
        let provisional_expansion = plan.provisional_expand(&ctx.scenario).map_err(|error| {
            ctx.eeg.log(format!(
                "[FollowRoute] Provisional expansion error from {} - {:?}",
                error.planner_name, error.error,
            ));
            for line in log.iter().chain(&error.log) {
                ctx.eeg.log(format!("[FollowRoute] {}", line));
            }
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
