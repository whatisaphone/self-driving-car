use crate::routing::models::CarState;
use common::{prelude::*, rl};
use derive_new::new;
use nalgebra::Point2;
use simulate::Car1D;

#[derive(new)]
pub struct SimGroundDrive {
    target_loc: Point2<f32>,
}

impl SimGroundDrive {
    pub fn simulate(&self, start: &CarState, time: f32, throttle: f32, boost: bool) -> CarState {
        let mut car = Car1D::new()
            .with_speed(start.vel.norm())
            .with_boost(start.boost as f32);
        car.advance(time, throttle, boost);

        let axis = (self.target_loc - start.loc_2d()).to_axis();
        let loc = start.loc_2d() + axis.as_ref() * car.distance();
        let vel = axis.as_ref() * car.speed();

        CarState {
            loc: loc.to_3d(rl::OCTANE_NEUTRAL_Z),
            rot: start.rot,
            vel: vel.to_3d(0.0),
            boost: car.boost(),
        }
    }
}
