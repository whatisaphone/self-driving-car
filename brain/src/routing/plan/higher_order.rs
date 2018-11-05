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
