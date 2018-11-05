use routing::models::{PlanningContext, RoutePlan, RoutePlanError, RoutePlanner};

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

    fn plan(&self, _ctx: &PlanningContext) -> Result<RoutePlan, RoutePlanError> {
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

    fn plan(&self, ctx: &PlanningContext) -> Result<RoutePlan, RoutePlanError> {
        let plan = self.head.plan(ctx)?;
        Ok(Self::join_planner(plan, self.next.clone()))
    }
}

impl ChainedPlanner {
    /// Create a plan that runs `first` to completion, and then runs `second`.
    pub fn join_planner(first: RoutePlan, second: Option<Box<RoutePlanner>>) -> RoutePlan {
        let next: Option<Box<RoutePlanner>> = match first.next {
            Some(first_next) => Some(Box::new(ChainedPlanner::new(first_next, second))),
            None => second,
        };
        RoutePlan {
            segment: first.segment,
            next,
        }
    }

    pub fn chain(planners: Vec<Box<RoutePlanner>>) -> Box<RoutePlanner> {
        let mut iter = planners.into_iter();
        let mut result = iter.next().unwrap();
        for planner in iter {
            result = Box::new(Self::new(result, Some(planner)));
        }
        result
    }
}
