use crate::car1d::Car1D;
use common::{physics, prelude::*};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector2, Vector3};

pub struct Car {
    loc: Point3<f32>,
    rot: UnitQuaternion<f32>,
    vel: Vector3<f32>,
    boost: f32,
    on_ground: bool,
}

#[derive(Debug)]
pub enum CarSimulateError {
    NotOnGround,
    Skidding,
}

impl Car {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            loc: Point3::origin(),
            rot: UnitQuaternion::identity(),
            vel: Vector3::zeros(),
            boost: 100.0,
            on_ground: true,
        }
    }

    pub fn from_player_info(player: &common::halfway_house::PlayerInfo) -> Self {
        Self {
            loc: player.Physics.loc(),
            rot: player.Physics.quat(),
            vel: player.Physics.vel(),
            boost: player.Boost as f32,
            on_ground: player.OnGround,
        }
    }

    pub fn loc(&self) -> Point3<f32> {
        self.loc
    }

    pub fn rot(&self) -> UnitQuaternion<f32> {
        self.rot
    }

    pub fn vel(&self) -> Vector3<f32> {
        self.vel
    }

    pub fn step_throttle_boost(
        &mut self,
        dt: f32,
        throttle: f32,
        boost: bool,
    ) -> Result<(), CarSimulateError> {
        if !self.on_ground {
            return Err(CarSimulateError::NotOnGround);
        }

        // If we are skidding, this method will give incorrect results. Once chip's
        // simulate is integrated, consider raising the 0.95 to 0.98 or 0.99 and
        // cutting over sooner.
        if !(self.vel.norm() <= 100.0 || self.vel.normalize().dot(&self.forward()) >= 0.95) {
            return Err(CarSimulateError::Skidding);
        }

        let mut car1d = Car1D::new()
            .with_speed(self.vel.norm())
            .with_boost(self.boost);
        car1d.advance(dt, throttle, boost);

        self.loc += self.forward().into_inner() * car1d.distance();
        self.vel = self.forward().into_inner() * car1d.speed();
        self.boost = car1d.boost();
        Ok(())
    }

    /// A unit vector in the forward direction.
    pub fn forward(&self) -> Unit<Vector3<f32>> {
        physics::car_forward_axis(self.rot)
    }

    /// A unit vector in the forward direction in 2D.
    pub fn forward_axis_2d(&self) -> Unit<Vector2<f32>> {
        physics::car_forward_axis_2d(self.rot.to_2d())
    }
}
