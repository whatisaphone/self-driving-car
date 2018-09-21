use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use ncollide3d::{
    query::Ray,
    shape::{Plane, ShapeHandle},
    world::{CollisionGroups, CollisionWorld, GeometricQueryType},
};
use simulate::rl;
use utils::{ExtendVector2, TotalF32};

lazy_static! {
    pub static ref WALL_RAY_CALCULATOR: WallRayCalculator = WallRayCalculator::new();
}

pub struct WallRayCalculator {
    world: CollisionWorld<f32, ()>,
}

impl WallRayCalculator {
    fn new() -> WallRayCalculator {
        let mut fixed = CollisionGroups::new();
        fixed.set_membership(&[0]);

        let mut world = CollisionWorld::new(1.0);
        let exact = GeometricQueryType::Contacts(0.0, 0.0);
        world.add(
            Isometry3::new(Vector3::new(-rl::FIELD_MAX_X, 0.0, 0.0), Vector3::zeros()),
            ShapeHandle::new(Plane::new(Vector3::x_axis())),
            fixed,
            exact,
            (),
        );
        world.add(
            Isometry3::new(Vector3::new(0.0, -rl::FIELD_MAX_Y, 0.0), Vector3::zeros()),
            ShapeHandle::new(Plane::new(Vector3::y_axis())),
            fixed,
            exact,
            (),
        );
        world.add(
            Isometry3::new(Vector3::new(rl::FIELD_MAX_X, 0.0, 0.0), Vector3::zeros()),
            ShapeHandle::new(Plane::new(-Vector3::x_axis())),
            fixed,
            exact,
            (),
        );
        world.add(
            Isometry3::new(Vector3::new(0.0, rl::FIELD_MAX_Y, 0.0), Vector3::zeros()),
            ShapeHandle::new(Plane::new(-Vector3::y_axis())),
            fixed,
            exact,
            (),
        );
        world.update();
        WallRayCalculator { world }
    }

    pub fn calculate(&self, from: Vector2<f32>, to: Vector2<f32>) -> Point3<f32> {
        let ray = Ray::new(
            Point3::from_coordinates(from.to_3d(0.0)),
            (to - from).to_3d(0.0),
        );
        let (_, intersect) = self
            .world
            .interferences_with_ray(&ray, &CollisionGroups::new())
            .filter(|(cobj, _)| {
                // Ignore walls that the `from` point is "behind"
                if cobj.position().translation.vector.y == -rl::FIELD_MAX_Y
                    && from.y < -rl::FIELD_MAX_Y
                {
                    return false;
                }
                if cobj.position().translation.vector.y == rl::FIELD_MAX_Y
                    && from.y > rl::FIELD_MAX_Y
                {
                    return false;
                }
                true
            }).min_by_key(|(_, intersect)| TotalF32(intersect.toi))
            .unwrap();
        ray.origin + ray.dir * intersect.toi
    }
}
