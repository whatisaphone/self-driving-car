use crate::{
    behavior::{
        higher_order::Chain,
        movement::{drive_towards, simple_yaw_diff, GetToFlatGround},
    },
    eeg::{color, Drawable},
    plan::drive::get_route_dodge,
    strategy::{Action, Behavior, Context},
};
use common::{prelude::*, rl};
use nalgebra::Point2;
use nameof::name_of_type;
use rlbot;
use simulate::Car1D;
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
        name_of_type!(GroundAccelToLoc)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();
        let distance = (me.Physics.loc_2d() - self.target_loc).norm();
        let time_remaining = self.target_time - ctx.packet.GameInfo.TimeSeconds;

        ctx.eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
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
            return Action::call(Chain::new(self.priority(), vec![
                dodge,
                Box::new(Self { ..*self }),
            ]));
        }

        let mut result = drive_towards(ctx, self.target_loc);

        let yaw_diff = simple_yaw_diff(&me.Physics, self.target_loc);
        if yaw_diff.abs() > PI / 8.0 && distance >= 500.0 {
            // Can't estimate accurately if not facing the right way. Just return the result
            // with throttle == 1.0.
            return Action::Yield(result);
        }

        let (throttle, boost) = estimate_approach(&me, distance, time_remaining - 2.0 / 120.0);
        result.Throttle = throttle;
        result.Boost = boost
            && me.OnGround
            && yaw_diff.abs() < PI / 4.0
            && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED
            && me.Boost > 0;
        Action::Yield(result)
    }
}

/// Starting at `origin`, if we go pedal to the metal for `time` seconds, will
/// we have traveled `distance`?
#[allow(clippy::if_same_then_else)]
fn estimate_approach(car: &rlbot::ffi::PlayerInfo, distance: f32, time: f32) -> (f32, bool) {
    let would_reach = |throttle, boost| {
        let lag_comp = 1.5 / 120.0; // Start a few ticks later to compensate for input lag.
        let mut sim_car = Car1D::new()
            .with_speed(car.Physics.vel().norm())
            .with_boost(car.Boost as f32);
        sim_car.advance(time - lag_comp, throttle, boost);
        sim_car.distance() >= distance
    };

    if would_reach(0.0, false) {
        (0.0, false) // We're overshooting…
    } else if would_reach(1.0, false) {
        (0.0, false)
    } else if would_reach(1.0, true) {
        (1.0, false)
    } else {
        (1.0, true)
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::movement::GroundAccelToLoc,
        integration_tests::helpers::{TestRunner, TestScenario},
    };
    use common::prelude::*;
    use nalgebra::{Point2, Point3};

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
                    ball_loc: Point3::new(2000.0, 0.0, 0.0),
                    boost,
                    ..Default::default()
                },
                move |p| GroundAccelToLoc::new(target_loc, p.GameInfo.TimeSeconds + 2.0),
            );

            test.sleep_millis(2000);

            let packet = test.sniff_packet();
            let diff = (packet.GameCars[0].Physics.loc_2d() - target_loc).norm();
            println!("target loc: {:.?}", target_loc);
            println!("car loc: {:.?}", packet.GameCars[0].Physics.loc());
            println!("diff: {:.0}", diff);
            assert!(diff.abs() < 20.0);
        }
    }
}
