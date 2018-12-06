use crate::{
    behavior::{defensive_hit, Action, Behavior, Chain, Priority},
    maneuvers::GroundedHit,
    routing::{behavior::FollowRoute, plan::GroundIntercept},
    strategy::Context,
    utils::geometry::ExtendF32,
};
use common::prelude::*;
use nalgebra::Point2;

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

/// Calculate an angle from `ball_loc` to `car_loc`, trying to get between
/// `ball_loc` and `block_loc`, but not adjusting the approach angle by more
/// than `max_angle_diff`.
pub fn blocking_angle(
    ball_loc: Point2<f32>,
    car_loc: Point2<f32>,
    block_loc: Point2<f32>,
    max_angle_diff: f32,
) -> f32 {
    let naive_angle = ball_loc.coords.angle_to(car_loc.coords);
    let block_angle = ball_loc.coords.angle_to(block_loc.coords);
    let adjust = (block_angle - naive_angle)
        .normalize_angle()
        .max(-max_angle_diff)
        .min(max_angle_diff);
    (naive_angle + adjust).normalize_angle()
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
