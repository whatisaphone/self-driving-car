use behavior::{Action, Behavior};
use routing::models::{RoutePlanError, RoutePlanner, RouteStep, SegmentRunAction, SegmentRunner};
use strategy::Context;

pub struct FollowRoute {
    /// Option dance: This only holds a planner before the first tick.
    planner: Option<Box<RoutePlanner>>,
    cur_step: Option<RouteStep>,
    cur_runner: Option<Box<SegmentRunner>>,
}

impl FollowRoute {
    pub fn new(planner: impl RoutePlanner + 'static) -> Self {
        Self {
            planner: Some(Box::new(planner)),
            cur_step: None,
            cur_runner: None,
        }
    }
}

impl Behavior for FollowRoute {
    fn name(&self) -> &str {
        stringify!(FollowRoute)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if self.cur_step.is_none() {
            let planner = &*self.planner.take().unwrap();
            if let Err(action) = self.advance(planner, ctx) {
                return action;
            }
        }

        if self.draw(ctx).is_err() {
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
        let cur_step = self.cur_step.as_ref().unwrap();
        let expansion = cur_step.provisional_expand(&ctx.scenario)?;

        for segment in expansion.iter() {
            segment.draw(ctx);
        }
        Ok(())
    }

    fn advance(&mut self, planner: &RoutePlanner, ctx: &mut Context) -> Result<(), Action> {
        let step = match planner.plan(&ctx.me().into(), &ctx.scenario) {
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
        let runner = step.segment.run();

        self.cur_step = Some(step);
        self.cur_runner = Some(runner);
        Ok(())
    }

    fn go(&mut self, ctx: &mut Context) -> Action {
        let action = {
            let cur_runner = self.cur_runner.as_mut().unwrap();
            cur_runner.execute(ctx)
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

        ctx.eeg.log("[FollowRoute] Next segment");
        let cur_step = self.cur_step.take().unwrap();
        let next = some_or_else!(cur_step.next, {
            return Action::Return;
        });
        if let Err(action) = self.advance(&*next, ctx) {
            return action;
        }
        self.go(ctx)
    }
}
