use behavior::Behavior;
use routing::{
    behavior::FollowRoute,
    models::{RoutePlan, SegmentPlan},
};

pub fn segment_plan_tester(plan: impl SegmentPlan + 'static) -> impl Behavior {
    FollowRoute::new(RoutePlan::new(vec![Box::new(plan)]))
}
