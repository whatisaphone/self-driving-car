use common::prelude::*;
use euclid::{TypedPoint3D, TypedVector3D};
use nalgebra::{Point2, Point3, Unit, Vector2, Vector3};
use plane_split::{Line as TypedLine, Plane as TypedPlane};
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

#[derive(Copy, Clone)]
pub struct Line {
    pub origin: Point3<f32>,
    pub dir: Unit<Vector3<f32>>,
}

impl From<TypedLine<f32, ()>> for Line {
    fn from(line: TypedLine<f32, ()>) -> Self {
        Line {
            origin: point3_from_euclid(line.origin),
            dir: Unit::new_unchecked(vector3_from_euclid(line.dir)),
        }
    }
}

#[derive(Copy, Clone)]
pub struct Plane {
    pub normal: Unit<Vector3<f32>>,
    pub offset: f32,
}

impl Plane {
    /// Constructs a plane from point-normal form.
    pub fn point_normal(p: Point3<f32>, n: Unit<Vector3<f32>>) -> Self {
        let offset = -(n.x * p.x + n.y * p.y + n.z * p.z);
        Plane { normal: n, offset }
    }

    pub fn distance_to_point(&self, point: Point3<f32>) -> f32 {
        let plane = TypedPlane::from(*self);
        plane.signed_distance_to(&point3_to_euclid(point))
    }

    pub fn intersect(&self, other: &Self) -> Option<Line> {
        let plane = TypedPlane::from(*self);
        let other = TypedPlane::from(*other);
        plane.intersect(&other).map(Into::into)
    }
}

impl From<TypedPlane<f32, ()>> for Plane {
    fn from(plane: TypedPlane<f32, ()>) -> Self {
        Self {
            normal: unit_vector3_from_euclid(plane.normal),
            offset: plane.offset,
        }
    }
}

impl From<Plane> for TypedPlane<f32, ()> {
    fn from(plane: Plane) -> Self {
        Self {
            normal: unit_vector3_to_euclid(plane.normal),
            offset: plane.offset,
        }
    }
}

fn vector3_from_euclid(v: TypedVector3D<f32, ()>) -> Vector3<f32> {
    Vector3::new(v.x, v.y, v.z)
}

fn unit_vector3_from_euclid(v: TypedVector3D<f32, ()>) -> Unit<Vector3<f32>> {
    Unit::new_unchecked(Vector3::new(v.x, v.y, v.z))
}

fn unit_vector3_to_euclid(v: Unit<Vector3<f32>>) -> TypedVector3D<f32, ()> {
    TypedVector3D::new(v.x, v.y, v.z)
}

fn point3_from_euclid(v: TypedPoint3D<f32, ()>) -> Point3<f32> {
    Point3::new(v.x, v.y, v.z)
}

fn point3_to_euclid(point: Point3<f32>) -> TypedPoint3D<f32, ()> {
    TypedPoint3D::new(point.x, point.y, point.z)
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
