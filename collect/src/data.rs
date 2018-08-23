use nalgebra::{Rotation3, Vector3};
use utils::ExtendRotation3;

pub struct Snapshot {
    pub time: f32,
    pub ball: Physics,
    pub cars: Vec<Physics>,
}

pub struct Physics {
    pub loc: Vector3<f32>,
    pub rot: Rotation3<f32>,
    pub vel: Vector3<f32>,
    pub ang_vel: Vector3<f32>,
}

impl Snapshot {
    pub fn from_row(mut it: impl Iterator<Item = f32>) -> Result<Self, ()> {
        let time = it.next().ok_or(())?;
        let ball = to_physics(&mut it)?;
        let mut cars = Vec::new();
        loop {
            match to_physics(&mut it) {
                Ok(car) => cars.push(car),
                Err(_) => break,
            }
        }
        Ok(Snapshot { time, ball, cars })
    }
}

fn to_physics(it: &mut impl Iterator<Item = f32>) -> Result<Physics, ()> {
    Ok(Physics {
        loc: to_vector3(it)?,
        rot: to_rotation(it)?,
        vel: to_vector3(it)?,
        ang_vel: to_vector3(it)?,
    })
}

fn to_vector3(it: &mut impl Iterator<Item = f32>) -> Result<Vector3<f32>, ()> {
    Ok(Vector3::new(
        it.next().ok_or(())?,
        it.next().ok_or(())?,
        it.next().ok_or(())?,
    ))
}

fn to_rotation(it: &mut impl Iterator<Item = f32>) -> Result<Rotation3<f32>, ()> {
    Ok(Rotation3::from_unreal_angles(
        it.next().ok_or(())?,
        it.next().ok_or(())?,
        it.next().ok_or(())?,
    ))
}
