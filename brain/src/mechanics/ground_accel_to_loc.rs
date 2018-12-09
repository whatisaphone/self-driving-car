use crate::{
    behavior::{Action, Behavior, Chain},
    eeg::{color, Drawable},
    maneuvers::{drive_towards, GetToFlatGround},
    mechanics::simple_yaw_diff,
    plan::drive::get_route_dodge,
    strategy::Context,
};
use common::{prelude::*, rl};
use nalgebra::Point2;
use rlbot;
use simulate::Car1Dv2;
use std::f32::consts::PI;

pub struct GroundAccelToLoc {
    target_loc: Point2<f32>,
    target_time: f32,
}

impl GroundAccelToLoc {
    pub fn new(target_loc: Point2<f32>, target_time: f32) -> GroundAccelToLoc {
        GroundAccelToLoc {
            target_loc,
            target_time,
        }
    }
}

impl Behavior for GroundAccelToLoc {
    fn name(&self) -> &str {
        stringify!(GroundAccelToLoc)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();
        let distance = (me.Physics.locp().to_2d() - self.target_loc).norm();
        let time_remaining = self.target_time - ctx.packet.GameInfo.TimeSeconds;

        ctx.eeg.draw(Drawable::ghost_car_ground(
            Point2::from(self.target_loc),
            me.Physics.rot(),
        ));
        ctx.eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));

        // This behavior currently just operates in 2D
        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            ctx.eeg.log("[GroudAccelToLoc] not on flat ground");
            return Action::Abort;
        }

        // A bit sloppy reasoning here
        if let Some(dodge) = get_route_dodge(me, self.target_loc) {
            // Dodge, then continue with `self`
            return Action::call(Chain::new(
                self.priority(),
                vec![dodge, Box::new(Self { ..*self })],
            ));
        }

        let yaw_diff = simple_yaw_diff(&me.Physics, self.target_loc);
        let too_fast = estimate_approach(&me, distance, time_remaining - 2.0 / 120.0);

        let mut result = drive_towards(ctx, self.target_loc);
        if yaw_diff.abs() > PI / 8.0 && distance >= 500.0 {
            // Can't estimate accurately if not facing the right way
        } else if too_fast {
            result.Throttle = 0.0;
        } else {
            if me.OnGround
                && yaw_diff.abs() < PI / 4.0
                && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED
                && me.Boost > 0
            {
                result.Boost = true;
            }
        }

        Action::Yield(result)
    }
}

/// Starting at `origin`, if we go pedal to the metal for `time` seconds, will
/// we have traveled `distance`?
fn estimate_approach(car: &rlbot::ffi::PlayerInfo, distance: f32, time: f32) -> bool {
    let lag_comp = 1.5 / 120.0; // Start a few ticks later to compensate for input lag.
    let mut sim_car = Car1Dv2::new()
        .with_speed(car.Physics.vel().norm())
        .with_boost(car.Boost as f32);
    sim_car.advance(time - lag_comp, 1.0, true);
    sim_car.distance() >= distance
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        integration_tests::helpers::{TestRunner, TestScenario},
        mechanics::GroundAccelToLoc,
    };
    use common::prelude::*;
    use nalgebra::{Point2, Vector3};

    // This test is ignored because it's finicky and not quite accurate. The
    // issue seems to be that there is more input lag for throttle than for
    // boost? Weird stuff. Anyhow, this test is a nice ideal at least :)
    #[test]
    #[ignore]
    fn verify_arrival_time() {
        let cases = [(-200.0, 500.0, 0), (100.0, 600.0, 50)];
        for &(x, y, boost) in cases.iter() {
            let target_loc = Point2::new(x, y);
            let test = TestRunner::start2(
                TestScenario {
                    ball_loc: Vector3::new(2000.0, 0.0, 0.0),
                    boost,
                    ..Default::default()
                },
                move |p| GroundAccelToLoc::new(target_loc, p.GameInfo.TimeSeconds + 2.0),
            );

            test.sleep_millis(2000);

            let packet = test.sniff_packet();
            let diff = (packet.GameCars[0].Physics.locp().to_2d() - target_loc).norm();
            println!("target loc: {:.?}", target_loc);
            println!("car loc: {:.?}", packet.GameCars[0].Physics.locp());
            println!("diff: {:.0}", diff);
            assert!(diff.abs() < 20.0);
        }
    }
}
