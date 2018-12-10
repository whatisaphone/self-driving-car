use crate::{
    behavior::{defensive_hit, Action, Behavior, Chain, Priority},
    maneuvers::GroundedHit,
    routing::{behavior::FollowRoute, plan::GroundIntercept},
    strategy::Context,
};

pub struct Kickoff;

impl Kickoff {
    pub fn new() -> Self {
        Kickoff
    }
}

impl Behavior for Kickoff {
    fn name(&self) -> &str {
        stringify!(Kickoff)
    }

    fn execute2(&mut self, _ctx: &mut Context) -> Action {
        Action::call(Chain::new(
            Priority::Idle,
            vec![
                Box::new(FollowRoute::new(
                    GroundIntercept::new().allow_dodging(false),
                )),
                Box::new(GroundedHit::hit_towards(defensive_hit)),
            ],
        ))
    }
}
