use nalgebra::{Point2, Point3, Real, UnitComplex, UnitQuaternion, Vector2, Vector3};
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

pub trait ExtendVector2 {
    fn unit(angle: f32) -> Self;
    fn to_3d(&self, z: f32) -> Vector3<f32>;
    fn uc_angle_to(&self, other: Self) -> UnitComplex<f32>;
    fn angle_to(&self, other: Self) -> f32;
    fn rotation_to(&self, other: Self) -> UnitComplex<f32>;
}

impl ExtendVector2 for Vector2<f32> {
    fn unit(angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Vector2::new(cos, sin)
    }

    fn to_3d(&self, z: f32) -> Vector3<f32> {
        Vector3::new(self.x, self.y, z)
    }

    // This treats `Vector`s as `Point`s. It should be deprecated.
    fn uc_angle_to(&self, other: Self) -> UnitComplex<f32> {
        let diff = other - self;
        UnitComplex::new(f32::atan2(diff.y, diff.x))
    }

    // This treats `Vector`s as `Point`s. It should be deprecated.
    fn angle_to(&self, other: Self) -> f32 {
        let diff = other - self;
        f32::atan2(diff.y, diff.x)
    }

    fn rotation_to(&self, other: Self) -> UnitComplex<f32> {
        UnitComplex::rotation_between(self, &other)
    }
}

pub trait ExtendVector3 {
    fn to_2d(&self) -> Vector2<f32>;
}

impl ExtendVector3 for Vector3<f32> {
    fn to_2d(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }
}

pub trait ExtendPoint2<N: Real> {
    fn angle_to(&self, other: Self) -> UnitComplex<N>;
    fn to_3d(&self, z: N) -> Point3<N>;
}

impl<N: Real> ExtendPoint2<N> for Point2<N> {
    fn angle_to(&self, other: Self) -> UnitComplex<N> {
        let diff = other - self;
        UnitComplex::new(N::atan2(diff.y, diff.x))
    }

    fn to_3d(&self, z: N) -> Point3<N> {
        Point3::new(self.x, self.y, z)
    }
}

pub trait ExtendPoint3<N: Real> {
    fn to_2d(&self) -> Point2<N>;
}

impl<N: Real> ExtendPoint3<N> for Point3<N> {
    fn to_2d(&self) -> Point2<N> {
        Point2::new(self.x, self.y)
    }
}

pub trait ExtendUnitComplex {
    fn unit(&self) -> Vector2<f32>;
    /// Convert this complex number (representing a 2D rotation) into a unit
    /// quaternion representing a 3D rotation around the z-axis.
    fn around_z_axis(&self) -> UnitQuaternion<f32>;
}

impl ExtendUnitComplex for UnitComplex<f32> {
    fn unit(&self) -> Vector2<f32> {
        Vector2::new(self.cos_angle(), self.sin_angle())
    }

    fn around_z_axis(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.angle())
    }
}

pub trait ExtendUnitQuaternion {
    fn to_2d(&self) -> UnitComplex<f32>;
}

impl ExtendUnitQuaternion for UnitQuaternion<f32> {
    fn to_2d(&self) -> UnitComplex<f32> {
        UnitComplex::new(self.scaled_axis().z)
    }
}

/// Returns the two points on a circle that form a tangent with the given point.
///
/// If the point is inside the circle, returns `None`.
pub fn circle_point_tangents(
    center: Point2<f32>,
    radius: f32,
    point: Point2<f32>,
) -> Option<[Point2<f32>; 2]> {
    // I'm so glad the internet exists
    // http://www.ambrsoft.com/TrigoCalc/Circles2/CirclePoint/CirclePointDistance.htm

    let a = center.x;
    let b = center.y;
    let r = radius;
    let xp = point.x;
    let yp = point.y;

    let xpm = r * (yp - b) * ((xp - a).powi(2) + (yp - b).powi(2) - r.powi(2)).sqrt();
    let x1 = (r.powi(2) * (xp - a) + xpm) / ((xp - a).powi(2) + (yp - b).powi(2)) + a;
    let x2 = (r.powi(2) * (xp - a) - xpm) / ((xp - a).powi(2) + (yp - b).powi(2)) + a;

    let ymp = r * (xp - a) * ((xp - a).powi(2) + (yp - b).powi(2) - r.powi(2)).sqrt();
    let y1 = (r.powi(2) * (yp - b) - ymp) / ((xp - a).powi(2) + (yp - b).powi(2)) + b;
    let y2 = (r.powi(2) * (yp - b) + ymp) / ((xp - a).powi(2) + (yp - b).powi(2)) + b;

    if x1.is_nan() {
        None
    } else {
        Some([Point2::new(x1, y1), Point2::new(x2, y2)])
    }
}
