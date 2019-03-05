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
        Self
    }
}

impl Behavior for FiftyFifty {
    fn name(&self) -> &str {
        name_of_type!(FiftyFifty)
    }

    fn execute_old(&mut self, _ctx: &mut Context<'_>) -> Action {
        Action::tail_call(While::new(
            WeDontWinTheRace,
            Chain::new(Priority::Idle, vec![
                Box::new(FollowRoute::new(GroundIntercept::new()).same_ball_trajectory(true)),
                Box::new(GroundedHit::hit_towards(defensive_hit)),
            ]),
        ))
    }
}
