use behavior::{Action, Behavior};
use routing::models::{RoutePlanError, RoutePlanner, RouteStep, SegmentRunAction, SegmentRunner};
use strategy::Context;

pub struct FollowRoute {
    /// Option dance: This only holds a planner before the first tick.
    planner: Option<Box<RoutePlanner>>,
    current: Option<Current>,
}

struct Current {
    step: RouteStep,
    duration: f32,
    start: f32,
    runner: Box<SegmentRunner>,
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
        let cur_end = current.start + current.duration;
        let expand_start_time = (cur_end - ctx.packet.GameInfo.TimeSeconds).max(0.0);
        let expansion = current
            .step
            .provisional_expand(expand_start_time, &ctx.scenario)?;

        for segment in expansion.iter() {
            segment.draw(ctx);
        }
        Ok(())
    }

    fn advance(&mut self, planner: &RoutePlanner, ctx: &mut Context) -> Result<(), Action> {
        assert!(self.current.is_none());

        let step = match planner.plan(0.0, &ctx.me().into(), &ctx.scenario) {
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
        let duration = step.segment.duration();

        self.current = Some(Current {
            step,
            duration,
            start: ctx.packet.GameInfo.TimeSeconds,
            runner,
        });
        Ok(())
    }

    fn go(&mut self, ctx: &mut Context) -> Action {
        let action = self.current.as_mut().unwrap().runner.execute(ctx);
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
        let next = some_or_else!(current.step.next, {
            return Action::Return;
        });
        ctx.eeg.log("[FollowRoute] Next segment");
        if let Err(action) = self.advance(&*next, ctx) {
            return action;
        }
        self.go(ctx)
    }
}
