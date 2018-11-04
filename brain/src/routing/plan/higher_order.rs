use routing::models::{CarState, RoutePlan, RoutePlanError, RoutePlanner};
use strategy::Scenario;

/// Exhaust `head`, then advance to `next`.
#[derive(Clone)]
pub struct StaticPlanner {
    plan: RoutePlan,
}

impl StaticPlanner {
    pub fn new(plan: RoutePlan) -> Self {
        Self { plan }
    }
}

impl RoutePlanner for StaticPlanner {
    fn name(&self) -> &'static str {
        stringify!(StaticPlanner)
    }

    fn plan(
        &self,
        _start_time: f32,
        _start: &CarState,
        _scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        Ok(self.plan.clone())
    }
}

/// Exhaust `head`, then advance to `next`.
#[derive(Clone, new)]
pub struct ChainedPlanner {
    head: Box<RoutePlanner>,
    next: Option<Box<RoutePlanner>>,
}

impl RoutePlanner for ChainedPlanner {
    fn name(&self) -> &'static str {
        stringify!(ChainedPlanner)
    }

    fn plan(
        &self,
        start_time: f32,
        start: &CarState,
        scenario: &Scenario,
    ) -> Result<RoutePlan, RoutePlanError> {
        let plan = self.head.plan(start_time, start, scenario)?;
        let next: Option<Box<RoutePlanner>> = match plan.next {
            Some(next) => Some(Box::new(ChainedPlanner::new(next, self.next.clone()))),
            None => self.next.clone(),
        };
        Ok(RoutePlan {
            segment: plan.segment,
            next,
        })
    }
}
