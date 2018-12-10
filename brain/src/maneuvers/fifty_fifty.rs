use crate::{
    behavior::{defensive_hit, Action, Behavior, Chain, Priority},
    maneuvers::GroundedHit,
    routing::{behavior::FollowRoute, plan::GroundIntercept},
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
        Action::call(Chain::new(
            Priority::Idle,
            vec![
                Box::new(FollowRoute::new(GroundIntercept::new())),
                Box::new(GroundedHit::hit_towards(defensive_hit)),
            ],
        ))
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        integration_tests::helpers::{TestRunner, TestScenario},
        maneuvers::FiftyFifty,
    };
    use nalgebra::Vector3;

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            FiftyFifty::new(),
            TestScenario {
                car_loc: Vector3::new(256.0, -3839.98, 17.01),
                ..Default::default()
            },
        );

        test.sleep_millis(5500);

        assert!(test.has_scored());
    }
}
