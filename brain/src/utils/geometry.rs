use common::prelude::*;
use nalgebra::{Point2, Unit, Vector2};
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

pub struct RayCoordinateSystem {
    origin: Point2<f32>,
    direction: Unit<Vector2<f32>>,
}

impl RayCoordinateSystem {
    pub fn segment(p: Point2<f32>, q: Point2<f32>) -> Self {
        Self {
            origin: p,
            direction: (q - p).to_axis(),
        }
    }

    pub fn project(&self, p: Point2<f32>) -> f32 {
        (p - self.origin).dot(&self.direction)
    }
}
