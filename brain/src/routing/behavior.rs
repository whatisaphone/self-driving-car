use behavior::{Action, Behavior};
use routing::models::{RoutePlan, SegmentRunAction, SegmentRunner};
use strategy::Context;

pub struct FollowRoute {
    plan: RoutePlan,
    cur_segment_index: usize,
    cur_segment: Box<SegmentRunner>,
}

impl FollowRoute {
    pub fn new(plan: RoutePlan) -> Self {
        let first_segment = plan.segments[0].run();
        Self {
            plan,
            cur_segment_index: 0,
            cur_segment: first_segment,
        }
    }
}

impl Behavior for FollowRoute {
    fn name(&self) -> &str {
        stringify!(FollowRoute)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        self.plan.draw(ctx);
        self.go(ctx)
    }
}

impl FollowRoute {
    fn go(&mut self, ctx: &mut Context) -> Action {
        let success = match self.cur_segment.execute(ctx) {
            SegmentRunAction::Yield(i) => return Action::Yield(i),
            SegmentRunAction::Success => true,
            SegmentRunAction::Failure => false,
        };

        if !success {
            ctx.eeg.log("[FollowRoute] Segment failure; aborting");
            return Action::Abort;
        }

        if self.cur_segment_index == self.plan.segments.len() - 1 {
            return Action::Return;
        }

        ctx.eeg.log("[FollowRoute] Next segment");
        self.cur_segment_index += 1;
        self.cur_segment = self.plan.segments[self.cur_segment_index].run();
        self.execute2(ctx)
    }
}
