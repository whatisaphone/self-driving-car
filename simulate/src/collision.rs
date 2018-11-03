use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use ncollide3d::{
    query::distance,
    shape::{Ball, Cuboid},
};

pub fn ball_car_distance(
    ball_loc: Point3<f32>,
    car_loc: Point3<f32>,
    car_rot: UnitQuaternion<f32>,
) -> f32 {
    let ball_iso = Isometry3::new(ball_loc.coords, Vector3::zeros());
    let ball = Ball::new(90.0);
    let car_pivot = Point3::origin(); // TODO
    let car_iso =
        Translation3::from(car_loc.coords) * Isometry3::rotation_wrt_point(car_rot, car_pivot);
    let car = Cuboid::new(Vector3::new(60.0, 40.0, 30.0)); // TODO

    distance(&ball_iso, &ball, &car_iso, &car)
}
