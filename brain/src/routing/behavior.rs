use crate::{
    behavior::{Action, Behavior},
    eeg::{color, Drawable},
    routing::models::{
        PlanningContext, PlanningDump, ProvisionalPlanExpansion, RoutePlan, RoutePlanError,
        RoutePlanner, SegmentRunAction, SegmentRunner,
    },
    strategy::Context,
};

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
        Self::new_boxed(Box::new(planner))
    }

    pub fn new_boxed(planner: Box<RoutePlanner>) -> Self {
        Self {
            planner: Some(planner),
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

        let context = PlanningContext {
            game: &ctx.game,
            start: ctx.me().into(),
            ball_prediction: ctx.scenario.ball_prediction(),
        };
        let mut log = Vec::new();
        let mut dump = PlanningDump { log: &mut log };
        ctx.eeg
            .log(format!("[FollowRoute] planning with {}", planner.name()));
        let plan = planner.plan(&context, &mut dump);
        let plan = match plan {
            Ok(s) => s,
            Err(err) => return Err(self.handle_error(ctx, planner.name(), err, log)),
        };
        ctx.eeg.log(format!(
            "[FollowRoute] next segment is {}",
            plan.segment.name(),
        ));
        let provisional_expansion = plan.provisional_expand(&ctx.scenario).map_err(|error| {
            self.handle_error(
                ctx,
                error.planner_name,
                error.error,
                log.into_iter().chain(error.log),
            )
        })?;

        let runner = plan.segment.run();
        self.current = Some(Current {
            plan,
            runner,
            provisional_expansion,
        });
        Ok(())
    }

    fn handle_error(
        &mut self,
        ctx: &mut Context,
        planner_name: &str,
        error: RoutePlanError,
        log: impl IntoIterator<Item = String>,
    ) -> Action {
        ctx.eeg.log(format!(
            "[FollowRoute] Error {:?} from segment {}",
            error, planner_name,
        ));
        for line in log {
            ctx.eeg.log(format!("[FollowRoute] {}", line));
        }

        match error.recover(ctx) {
            Some(b) => {
                ctx.eeg.log("[FollowRoute] Recoverable!");
                Action::Call(b)
            }
            None => {
                ctx.eeg.log("[FollowRoute] Non-recoverable");
                Action::Abort
            }
        }
    }

    fn go(&mut self, ctx: &mut Context) -> Action {
        let current = self.current.as_mut().unwrap();
        ctx.eeg
            .draw(Drawable::print(current.plan.segment.name(), color::YELLOW));

        let success = match current.runner.execute(ctx) {
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
        if let Err(action) = self.advance(&*next, ctx) {
            return action;
        }
        self.go(ctx)
    }
}
