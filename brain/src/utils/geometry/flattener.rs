use common::prelude::*;
use derive_new::new;
use nalgebra::{Isometry3, Point2, Point3, Unit, UnitComplex, UnitQuaternion, Vector2, Vector3};
use std::ops::Mul;

#[derive(Copy, Clone, new)]
pub struct Flattener {
    transform: Isometry3<f32>,
}

impl Flattener {
    pub fn identity() -> Self {
        Self::new(Isometry3::identity())
    }

    pub fn inverse(&self) -> Isometry3<f32> {
        self.transform.inverse()
    }
}

impl Mul<Point3<f32>> for Flattener {
    type Output = Point2<f32>;

    fn mul(self, rhs: Point3<f32>) -> Self::Output {
        (self.transform * rhs).xy()
    }
}

impl Mul<Vector3<f32>> for Flattener {
    type Output = Vector2<f32>;

    fn mul(self, rhs: Vector3<f32>) -> Self::Output {
        (self.transform * rhs).xy()
    }
}

impl Mul<Unit<Vector3<f32>>> for Flattener {
    type Output = Unit<Vector2<f32>>;

    fn mul(self, rhs: Unit<Vector3<f32>>) -> Self::Output {
        Unit::new_normalize((self.transform * rhs.unwrap()).xy())
    }
}

impl Mul<UnitQuaternion<f32>> for Flattener {
    type Output = UnitComplex<f32>;

    fn mul(self, rhs: UnitQuaternion<f32>) -> Self::Output {
        (self.transform.rotation * rhs).project_2d(&Vector3::z_axis())
    }
}
