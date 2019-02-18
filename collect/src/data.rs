use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use std::io::Read;

pub struct RecordingTick {
    pub time: f32,
    pub ball: RecordingRigidBodyState,
    pub players: Vec<RecordingPlayerTick>,
}

#[derive(Clone)]
pub struct RecordingPlayerTick {
    pub state: RecordingRigidBodyState,
    pub input: RecordingPlayerInput,
}

#[derive(Clone)]
pub struct RecordingRigidBodyState {
    pub loc: Point3<f32>,
    pub rot: UnitQuaternion<f32>,
    pub vel: Vector3<f32>,
    pub ang_vel: Vector3<f32>,
}

pub type RecordingPlayerInput = common::halfway_house::PlayerInput;

impl RecordingTick {
    pub fn parse(r: impl Read) -> impl Iterator<Item = RecordingTick> {
        let mut r = csv::Reader::from_reader(r);

        let headers = r.headers().unwrap().clone();
        let num_players = (0..)
            .skip_while(|i| headers.iter().any(|h| h == format!("player{}_loc_x", i)))
            .next()
            .unwrap();

        r.into_records().map(Result::unwrap).map(move |row| {
            let mut it = &mut row.into_iter();
            let time = csv_f32(it).unwrap();
            let ball = RecordingRigidBodyState::from_csv(&mut it).unwrap();
            let mut players = Vec::new();
            for _ in 0..num_players {
                players.push(RecordingPlayerTick {
                    input: csv_input(it).unwrap(),
                    state: RecordingRigidBodyState::from_csv(it).unwrap(),
                })
            }
            RecordingTick {
                time,
                ball,
                players,
            }
        })
    }
}

impl RecordingRigidBodyState {
    pub fn from_csv<'a>(it: &mut impl Iterator<Item = &'a str>) -> Result<Self, ()> {
        Ok(Self {
            loc: Point3::from(csv_vector3(it)?),
            rot: csv_quat(it)?,
            vel: csv_vector3(it)?,
            ang_vel: csv_vector3(it)?,
        })
    }
}

fn csv_input<'a>(it: &mut impl Iterator<Item = &'a str>) -> Result<RecordingPlayerInput, ()> {
    Ok(RecordingPlayerInput {
        Throttle: csv_f32(it)?,
        Steer: csv_f32(it)?,
        Pitch: csv_f32(it)?,
        Yaw: csv_f32(it)?,
        Roll: csv_f32(it)?,
        Jump: csv_bool(it)?,
        Boost: csv_bool(it)?,
        Handbrake: csv_bool(it)?,
    })
}

fn csv_vector3<'a>(it: &mut impl Iterator<Item = &'a str>) -> Result<Vector3<f32>, ()> {
    Ok(Vector3::new(csv_f32(it)?, csv_f32(it)?, csv_f32(it)?))
}

fn csv_quat<'a>(it: &mut impl Iterator<Item = &'a str>) -> Result<UnitQuaternion<f32>, ()> {
    let x = csv_f32(it)?;
    let y = csv_f32(it)?;
    let z = csv_f32(it)?;
    let w = csv_f32(it)?;
    Ok(UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z)))
}

fn csv_bool<'a>(it: &mut impl Iterator<Item = &'a str>) -> Result<bool, ()> {
    match it.next() {
        Some(s) if s == "true" => Ok(true),
        Some(s) if s == "false" => Ok(false),
        _ => Err(()),
    }
}

fn csv_f32<'a>(it: &mut impl Iterator<Item = &'a str>) -> Result<f32, ()> {
    it.next().ok_or(())?.parse().map_err(|_| ())
}
