use crate::routing::models::{
    PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner,
};
use derive_new::new;
use nameof::name_of_type;

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
        name_of_type!(StaticPlanner)
    }

    fn plan(
        &self,
        _ctx: &PlanningContext<'_, '_>,
        _dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        Ok(self.plan.clone())
    }
}

/// Exhaust `head`, then advance to `next`.
#[derive(Clone, new)]
pub struct ChainedPlanner {
    head: Box<dyn RoutePlanner>,
    next: Option<Box<dyn RoutePlanner>>,
}

impl RoutePlanner for ChainedPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(ChainedPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        let plan = self.head.plan(ctx, dump)?;
        Ok(Self::join_planner(plan, self.next.clone()))
    }
}

impl ChainedPlanner {
    /// Create a plan that runs `first` to completion, and then runs `second`.
    pub fn join_planner(first: RoutePlan, second: Option<Box<dyn RoutePlanner>>) -> RoutePlan {
        let next: Option<Box<dyn RoutePlanner>> = match first.next {
            Some(first_next) => Some(Box::new(ChainedPlanner::new(first_next, second))),
            None => second,
        };
        RoutePlan {
            segment: first.segment,
            next,
        }
    }

    pub fn chain(planners: Vec<Box<dyn RoutePlanner>>) -> Self {
        let mut iter = planners.into_iter();
        let first = iter.next().unwrap();
        let mut result = Self::new(first, Some(iter.next().unwrap()));
        for planner in iter {
            result = Self::new(Box::new(result), Some(planner));
        }
        result
    }
}
