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

#[cfg(test)]
mod integration_tests {
    use crate::{integration_tests::helpers::TestRunner, strategy::Runner2};
    use brain_test_data::recordings;
    use common::prelude::*;

    #[test]
    fn kickoff_center() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::KICKOFF_CENTER, 107.0)
            .behavior(Runner2::soccar())
            .run_for_millis(3000);

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.locp();
        let ball_vel = packet.GameBall.Physics.vel();
        let future = ball_loc + ball_vel * 3.0;
        assert!(future.y >= -5000.0);
    }
}
