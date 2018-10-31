use behavior::Behavior;
use routing::{
    behavior::FollowRoute,
    models::{CarState, RoutePlanError, RoutePlanner, RouteStep, SegmentPlan},
};
use strategy::Scenario;

pub fn segment_plan_tester(plan: impl SegmentPlan + Clone + 'static) -> impl Behavior {
    FollowRoute::new(CookedPlanner::new(plan))
}

#[derive(new, Clone)]
struct CookedPlanner<P>
where
    P: SegmentPlan + Clone + 'static,
{
    plan: P,
}

impl<P> RoutePlanner for CookedPlanner<P>
where
    P: SegmentPlan + Clone + 'static,
{
    fn plan(
        &self,
        _start_time: f32,
        _start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RouteStep, RoutePlanError> {
        Ok(RouteStep {
            segment: Box::new(self.plan.clone()),
            next: None,
        })
    }
}
