use nalgebra::{Point2, Point3, Real, UnitComplex};
use std::f32::consts::PI;

pub use common::ext::{ExtendPoint3, ExtendUnitComplex, ExtendVector2, ExtendVector3};

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
