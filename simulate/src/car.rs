use car1d::Car1D;
use common::ext::ExtendPhysics;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use rlbot;

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
    pub fn new() -> Self {
        Self {
            loc: Point3::origin(),
            rot: UnitQuaternion::identity(),
            vel: Vector3::zeros(),
            boost: 100.0,
            on_ground: true,
        }
    }

    pub fn from_player_info(player: &rlbot::ffi::PlayerInfo) -> Self {
        Self {
            loc: player.Physics.locp(),
            rot: player.Physics.quat(),
            vel: player.Physics.vel(),
            boost: player.Boost as f32,
            on_ground: player.OnGround,
        }
    }

    pub fn loc(&self) -> Point3<f32> {
        self.loc
    }

    pub fn vel(&self) -> Vector3<f32> {
        self.vel
    }

    pub fn rot(&self) -> UnitQuaternion<f32> {
        self.rot
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
        if !(self.vel.norm() <= 10.0 || self.vel.normalize().dot(&self.forward()) >= 0.95) {
            return Err(CarSimulateError::Skidding);
        }

        let mut car1d = Car1D::new(self.vel.norm()).with_boost_float(self.boost);
        car1d.step(dt, throttle, boost);

        self.loc += self.forward() * car1d.distance_traveled();
        self.vel = self.forward() * car1d.speed();
        self.boost = car1d.boost();
        Ok(())
    }

    /// A unit vector in the forward direction.
    pub fn forward(&self) -> Vector3<f32> {
        self.rot * Vector3::x()
    }
}
