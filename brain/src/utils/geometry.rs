use nalgebra::{Vector2, Vector3};
use std::f32::consts::PI;

pub trait ExtendF32 {
    /// Normalize an angle to between -PI and PI.
    fn normalize_angle(self) -> Self;
}

impl ExtendF32 for f32 {
    fn normalize_angle(self) -> Self {
        let result = self % (PI * 2.0);
        if result < -PI {
            result + (PI * 2.0)
        } else if result >= PI {
            result - (PI * 2.0)
        } else {
            result
        }
    }
}

pub trait ExtendVector3 {
    fn to_2d(&self) -> Vector2<f32>;
    fn angle_to(&self, other: &Self) -> f32;
}

impl ExtendVector3 for Vector3<f32> {
    fn to_2d(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }

    fn angle_to(&self, other: &Self) -> f32 {
        let diff = other - self;
        f32::atan2(diff.y, diff.x)
    }
}
