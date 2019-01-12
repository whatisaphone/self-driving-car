use crate::utils::geometry::Plane;
use common::rl;
use lazy_static::lazy_static;
use nalgebra::{Point3, Unit, Vector3};
use ordered_float::NotNan;

pub struct Pitch {
    planes: Vec<Plane>,
}

impl Pitch {
    pub fn closest_plane(&self, point: &Point3<f32>) -> &Plane {
        self.planes
            .iter()
            .min_by_key(|plane| NotNan::new(plane.distance_to_point(point)).unwrap())
            .unwrap()
    }

    pub fn ground(&self) -> &Plane {
        &self.planes[0]
    }
}

const CORNER_WALL_X: f32 = 3518.0;
const CORNER_WALL_Y: f32 = 4546.0;

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

            // Corner walls
            Plane::point_normal(
                Point3::new(-CORNER_WALL_X, -CORNER_WALL_Y, 0.0),
                Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0)),
            ),
            Plane::point_normal(
                Point3::new(CORNER_WALL_X, -CORNER_WALL_Y, 0.0),
                Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0)),
            ),
            Plane::point_normal(
                Point3::new(-CORNER_WALL_X, CORNER_WALL_Y, 0.0),
                Unit::new_normalize(Vector3::new(1.0, -1.0, 0.0)),
            ),
            Plane::point_normal(
                Point3::new(CORNER_WALL_X, CORNER_WALL_Y, 0.0),
                Unit::new_normalize(Vector3::new(-1.0, -1.0, 0.0)),
            ),
        ],
    };
}
