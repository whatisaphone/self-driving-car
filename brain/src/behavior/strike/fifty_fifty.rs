use crate::{
    behavior::{
        defense::defensive_hit,
        higher_order::{Chain, While},
        strike::GroundedHit,
    },
    routing::{behavior::FollowRoute, plan::GroundIntercept, recover::WeDontWinTheRace},
    strategy::{Action, Behavior, Context, Priority},
};
use nameof::name_of_type;

pub struct FiftyFifty;

impl FiftyFifty {
    pub fn new() -> Self {
        FiftyFifty
    }
}

impl Behavior for FiftyFifty {
    fn name(&self) -> &str {
        name_of_type!(FiftyFifty)
    }

    fn execute(&mut self, _ctx: &mut Context) -> Action {
        Action::call(While::new(
            WeDontWinTheRace,
            Chain::new(Priority::Idle, vec![
                Box::new(FollowRoute::new(GroundIntercept::new())),
                Box::new(GroundedHit::hit_towards(defensive_hit)),
            ]),
        ))
    }
}
