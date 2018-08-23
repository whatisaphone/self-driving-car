use nalgebra::Vector3;
use std::f32::consts::PI;

/// Normalize an angle to between -pi and pi.
pub fn normalize_angle(theta: f32) -> f32 {
    let result = theta % (PI * 2.0);
    if result >= PI {
        result - (PI * 2.0)
    } else {
        result
    }
}

pub trait ExtendVector3 {
    fn angle_to(&self, other: &Self) -> f32;
}

impl ExtendVector3 for Vector3<f32> {
    fn angle_to(&self, other: &Self) -> f32 {
        let diff = other - self;
        f32::atan2(diff.y, diff.x)
    }
}
