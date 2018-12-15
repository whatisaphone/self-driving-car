use crate::{
    behavior::{defensive_hit, Action, Behavior, Chain, Priority, While},
    maneuvers::GroundedHit,
    routing::{behavior::FollowRoute, plan::GroundIntercept, recover::WeDontWinTheRace},
    strategy::Context,
};

pub struct FiftyFifty;

impl FiftyFifty {
    pub fn new() -> Self {
        FiftyFifty
    }
}

impl Behavior for FiftyFifty {
    fn name(&self) -> &str {
        stringify!(FiftyFifty)
    }

    fn execute2(&mut self, _ctx: &mut Context) -> Action {
        Action::call(While::new(
            WeDontWinTheRace,
            Chain::new(
                Priority::Idle,
                vec![
                    Box::new(FollowRoute::new(GroundIntercept::new())),
                    Box::new(GroundedHit::hit_towards(defensive_hit)),
                ],
            ),
        ))
    }
}
