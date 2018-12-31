use crate::utils::geometry::Plane;
use common::rl;
use lazy_static::lazy_static;
use nalgebra::{Point3, Vector3};
use ordered_float::NotNan;

pub struct Pitch {
    planes: Vec<Plane>,
}

impl Pitch {
    pub fn closest_plane(&self, point: Point3<f32>) -> &Plane {
        self.planes
            .iter()
            .min_by_key(|plane| NotNan::new(plane.distance_to_point(point)).unwrap())
            .unwrap()
    }
}

lazy_static! {
    /// I believe all soccar maps are the same as DFH Stadium.
    pub static ref DFH_STADIUM: Pitch = Pitch {
        planes: vec![
            // Floor and ceiling
            Plane::point_normal(Point3::origin(), Vector3::z_axis()),
            Plane::point_normal(Point3::new(0.0, 0.0, rl::FIELD_MAX_Z), -Vector3::z_axis()),

            // Walls
            Plane::point_normal(Point3::new(-rl::FIELD_MAX_X, 0.0, 0.0), Vector3::x_axis()),
            Plane::point_normal(Point3::new(rl::FIELD_MAX_X, 0.0, 0.0), -Vector3::x_axis()),
            Plane::point_normal(Point3::new(0.0, -rl::FIELD_MAX_X, 0.0), Vector3::y_axis()),
            Plane::point_normal(Point3::new(0.0, rl::FIELD_MAX_X, 0.0), -Vector3::y_axis()),
        ],
    };
}
