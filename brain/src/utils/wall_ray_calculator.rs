use common::{prelude::*, rl};
use nalgebra::{Isometry3, Point2, Vector2, Vector3};
use ncollide3d::{
    query::Ray,
    shape::{Plane, ShapeHandle},
    world::{CollisionGroups, CollisionWorld, GeometricQueryType},
};
use strategy::Game;
use utils::TotalF32;

lazy_static! {
    static ref WALL_RAY_CALCULATOR: WallRayCalculator = WallRayCalculator::new();
}

pub struct WallRayCalculator {
    world: CollisionWorld<f32, ()>,
}

impl WallRayCalculator {
    fn new() -> Self {
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
        Self { world }
    }

    pub fn calculate(from: Point2<f32>, to: Point2<f32>) -> Point2<f32> {
        let ray = Ray::new(from.to_3d(0.0), (to - from).to_3d(0.0));
        let (_, intersect) = WALL_RAY_CALCULATOR
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
            })
            .min_by_key(|(_, intersect)| TotalF32(intersect.toi))
            .unwrap();
        (ray.origin + ray.dir * intersect.toi).to_2d()
    }

    pub fn calc_ray(from: Point2<f32>, angle: f32) -> Vector2<f32> {
        Self::calculate(from, from + Vector2::unit(angle)).coords
    }

    pub fn wall_for_point(game: &Game, point: Point2<f32>) -> Wall {
        let to_enemy_goal = game.enemy_goal().center_2d - Point2::origin();
        let to_point = point - Point2::origin();

        // These are intentionally atan2(x, y), since the zero angle is on the y axis,
        // not the x axis.
        match to_enemy_goal.rotation_to(to_point).angle().abs() {
            a if a < f32::atan2(rl::GOALPOST_X, rl::FIELD_MAX_Y) => Wall::EnemyGoal,
            a if a < f32::atan2(rl::FIELD_MAX_X, rl::FIELD_MAX_Y) => Wall::EnemyBackWall,
            a if a < f32::atan2(rl::FIELD_MAX_X, -rl::FIELD_MAX_Y) => Wall::Midfield,
            a if a < f32::atan2(rl::GOALPOST_X, -rl::FIELD_MAX_Y) => Wall::OwnBackWall,
            _ => Wall::OwnGoal,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Wall {
    EnemyGoal,
    EnemyBackWall,
    Midfield,
    OwnBackWall,
    OwnGoal,
}
