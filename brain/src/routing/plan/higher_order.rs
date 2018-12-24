use crate::routing::models::{
    PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner,
};
use derive_new::new;

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
        _ctx: &PlanningContext,
        _dump: &mut PlanningDump,
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
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        let plan = self.head.plan(ctx, dump)?;
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

    pub fn chain(planners: Vec<Box<RoutePlanner>>) -> Self {
        let mut iter = planners.into_iter();
        let first = iter.next().unwrap();
        let mut result = Self::new(first, Some(iter.next().unwrap()));
        for planner in iter {
            result = Self::new(Box::new(result), Some(planner));
        }
        result
    }
}
