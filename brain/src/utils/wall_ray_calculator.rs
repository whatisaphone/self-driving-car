use crate::strategy::Game;
use common::{prelude::*, rl};
use lazy_static::lazy_static;
use nalgebra::{Isometry3, Point2, Vector2, Vector3};
use ncollide3d::{
    query::{Ray, RayCast},
    shape::Plane,
};
use ordered_float::NotNan;

lazy_static! {
    static ref WALL_RAY_CALCULATOR: WallRayCalculator = WallRayCalculator::new();
}

pub struct WallRayCalculator {
    walls: Vec<(Plane<f32>, Isometry3<f32>)>,
}

impl WallRayCalculator {
    fn new() -> Self {
        let walls = vec![
            (
                Plane::new(Vector3::x_axis()),
                Isometry3::new(Vector3::new(-rl::FIELD_MAX_X, 0.0, 0.0), Vector3::zeros()),
            ),
            (
                Plane::new(Vector3::y_axis()),
                Isometry3::new(Vector3::new(0.0, -rl::FIELD_MAX_Y, 0.0), Vector3::zeros()),
            ),
            (
                Plane::new(-Vector3::x_axis()),
                Isometry3::new(Vector3::new(rl::FIELD_MAX_X, 0.0, 0.0), Vector3::zeros()),
            ),
            (
                Plane::new(-Vector3::y_axis()),
                Isometry3::new(Vector3::new(0.0, rl::FIELD_MAX_Y, 0.0), Vector3::zeros()),
            ),
        ];
        Self { walls }
    }

    pub fn calculate(from: Point2<f32>, to: Point2<f32>) -> Point2<f32> {
        let ray = Ray::new(from.to_3d(0.0), (to - from).to_3d(0.0));
        let toi = WALL_RAY_CALCULATOR
            .walls
            .iter()
            .filter(|(_wall, m)| {
                // Ignore walls that the `from` point is "behind"
                if m.translation.vector.y == -rl::FIELD_MAX_Y && from.y < -rl::FIELD_MAX_Y {
                    return false;
                }
                if m.translation.vector.y == rl::FIELD_MAX_Y && from.y > rl::FIELD_MAX_Y {
                    return false;
                }
                true
            })
            .filter_map(|(wall, m)| wall.toi_with_ray(m, &ray, false))
            .min_by_key(|&toi| NotNan::new(toi).unwrap())
            .unwrap();
        (ray.origin + ray.dir * toi).to_2d()
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

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Wall {
    EnemyGoal,
    EnemyBackWall,
    Midfield,
    OwnBackWall,
    OwnGoal,
}
