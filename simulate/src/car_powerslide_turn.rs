use common::{
    ext::{ExtendUnitVector2, ExtendVector2},
    physics::{car_forward_axis_2d, CAR_LOCAL_FORWARD_AXIS_2D},
};
use nalgebra::{Point2, Unit, UnitComplex, Vector2};
use oven::data;

pub struct CarPowerslideTurn;

#[derive(Clone)]
pub struct CarPowerslideTurnPlan {
    pub start_loc: Point2<f32>,
    pub start_vel: Vector2<f32>,
    pub steer: f32,
    pub throttle: f32,
    pub end_loc: Point2<f32>,
    pub end_rot: UnitComplex<f32>,
    pub end_vel: Vector2<f32>,
    pub duration: f32,
}

impl CarPowerslideTurn {
    pub fn evaluate(
        start_loc: Point2<f32>,
        start_vel: Vector2<f32>,
        throttle: f32,
        target_dir: Unit<Vector2<f32>>,
    ) -> Option<CarPowerslideTurnPlan> {
        // Transform the input scenario to match the reference scenario. Then
        // afterwards, do the inverse transform to convert reference results back to the
        // input coordinate system.
        let reference_dir = Vector2::y_axis();
        let transform = start_vel.to_axis().rotation_to(&reference_dir);

        let start_rot = CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&start_vel.to_axis());
        let target_rot_by = car_forward_axis_2d(start_rot)
            .rotation_to(&target_dir)
            .angle();
        let steer = target_rot_by.signum();

        let reference = Self::reference_evaluate(start_vel.norm(), throttle, target_rot_by.abs());
        let reference = some_or_else!(reference, {
            return None;
        });

        // Transform the reference units back into input units.
        let mut reference_offset = reference.end_loc - reference.start_loc;
        reference_offset.x *= steer;
        let offset = transform.inverse() * reference_offset;

        Some(CarPowerslideTurnPlan {
            start_loc,
            start_vel,
            steer,
            throttle,
            end_loc: start_loc + offset,
            end_rot: CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&target_dir),
            end_vel: reference.end_vel,
            duration: reference.duration,
        })
    }

    /// Assume a car traveling at `start_speed`, which immediately hits the
    /// handbrake, turns right (steer 1.0), and sets throttle to `throttle`.
    /// Then wait for the car to rotate by `target_rot_by` radians. How long
    /// will it take, and how far will the car have traveled?
    fn reference_evaluate(
        start_speed: f32,
        throttle: f32,
        target_rot_by: f32,
    ) -> Option<CarPowerslideTurnPlan> {
        println!("start_speed: {:?}", start_speed);
        let speed_index = start_speed as usize / 100;
        println!("speed_index: {:?}", speed_index);
        println!("target_rot_by: {:?}", target_rot_by);
        // let lower_speed = speed_index as f32 * 100.0;
        let lower = TableSet::get(throttle, speed_index);
        // let upper_speed = ((speed_index + 1) as f32 * 100.0).min(rl::CAR_MAX_SPEED);
        let upper = TableSet::get(throttle, speed_index + 1);

        let start_rot = (lower.rot_2d_angle_cum[0] + upper.rot_2d_angle_cum[0]) / 2.0;
        let index = lower
            .rot_2d_angle_cum
            .iter()
            .zip(upper.rot_2d_angle_cum)
            .position(|(l, u)| (l + u) / 2.0 - start_rot >= target_rot_by);

        let index = some_or_else!(index, {
            return None;
        });
        let index = ((index as f32) * 0.5) as usize;

        // TODO: interpolate between `lower` and `upper`
        let start_time = lower.time[0];
        let end_time = lower.time[index];
        let start_loc = lower.loc_2d[0];
        let end_loc = lower.loc_2d[index];
        let start_rot = CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&Vector2::y_axis());
        let end_rot = UnitComplex::new(target_rot_by) * start_rot;
        let start_vel = lower.vel_2d[0];
        let end_vel = lower.vel_2d[index];
        Some(CarPowerslideTurnPlan {
            start_loc,
            start_vel,
            steer: 1.0,
            throttle,
            end_loc,
            end_rot,
            end_vel,
            duration: end_time - start_time,
        })
    }
}

struct TableSet<'a> {
    time: &'a [f32],
    loc_2d: &'a [Point2<f32>],
    rot_2d_angle_cum: &'a [f32],
    vel_2d: &'a [Vector2<f32>],
}

impl<'a> TableSet<'a> {
    fn get(throttle: f32, speed_index: usize) -> Self {
        if throttle == 0.0 {
            Self {
                time: POWERSLIDE_TURN_THROTTLE_0_TIME[speed_index],
                loc_2d: POWERSLIDE_TURN_THROTTLE_0_CAR_LOC_2D[speed_index],
                rot_2d_angle_cum: POWERSLIDE_TURN_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM[speed_index],
                vel_2d: POWERSLIDE_TURN_THROTTLE_0_CAR_VEL_2D[speed_index],
            }
        } else if throttle == 1.0 {
            Self {
                time: POWERSLIDE_TURN_THROTTLE_1_TIME[speed_index],
                loc_2d: POWERSLIDE_TURN_THROTTLE_1_CAR_LOC_2D[speed_index],
                rot_2d_angle_cum: POWERSLIDE_TURN_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM[speed_index],
                vel_2d: POWERSLIDE_TURN_THROTTLE_1_CAR_VEL_2D[speed_index],
            }
        } else {
            panic!("throttle not quantized")
        }
    }
}

const POWERSLIDE_TURN_THROTTLE_0_TIME: &[&[f32]] = &[
    data::powerslide_turn_speed_0_throttle_0::TIME,
    data::powerslide_turn_speed_100_throttle_0::TIME,
    data::powerslide_turn_speed_200_throttle_0::TIME,
    data::powerslide_turn_speed_300_throttle_0::TIME,
    data::powerslide_turn_speed_400_throttle_0::TIME,
    data::powerslide_turn_speed_500_throttle_0::TIME,
    data::powerslide_turn_speed_600_throttle_0::TIME,
    data::powerslide_turn_speed_700_throttle_0::TIME,
    data::powerslide_turn_speed_800_throttle_0::TIME,
    data::powerslide_turn_speed_900_throttle_0::TIME,
    data::powerslide_turn_speed_1000_throttle_0::TIME,
    data::powerslide_turn_speed_1100_throttle_0::TIME,
    data::powerslide_turn_speed_1200_throttle_0::TIME,
    data::powerslide_turn_speed_1300_throttle_0::TIME,
    data::powerslide_turn_speed_1400_throttle_0::TIME,
    data::powerslide_turn_speed_1500_throttle_0::TIME,
    data::powerslide_turn_speed_1600_throttle_0::TIME,
    data::powerslide_turn_speed_1700_throttle_0::TIME,
    data::powerslide_turn_speed_1800_throttle_0::TIME,
    data::powerslide_turn_speed_1900_throttle_0::TIME,
    data::powerslide_turn_speed_2000_throttle_0::TIME,
    data::powerslide_turn_speed_2100_throttle_0::TIME,
    data::powerslide_turn_speed_2200_throttle_0::TIME,
    data::powerslide_turn_speed_2299_98_throttle_0::TIME,
];

const POWERSLIDE_TURN_THROTTLE_1_TIME: &[&[f32]] = &[
    data::powerslide_turn_speed_0_throttle_1::TIME,
    data::powerslide_turn_speed_100_throttle_1::TIME,
    data::powerslide_turn_speed_200_throttle_1::TIME,
    data::powerslide_turn_speed_300_throttle_1::TIME,
    data::powerslide_turn_speed_400_throttle_1::TIME,
    data::powerslide_turn_speed_500_throttle_1::TIME,
    data::powerslide_turn_speed_600_throttle_1::TIME,
    data::powerslide_turn_speed_700_throttle_1::TIME,
    data::powerslide_turn_speed_800_throttle_1::TIME,
    data::powerslide_turn_speed_900_throttle_1::TIME,
    data::powerslide_turn_speed_1000_throttle_1::TIME,
    data::powerslide_turn_speed_1100_throttle_1::TIME,
    data::powerslide_turn_speed_1200_throttle_1::TIME,
    data::powerslide_turn_speed_1300_throttle_1::TIME,
    data::powerslide_turn_speed_1400_throttle_1::TIME,
    data::powerslide_turn_speed_1500_throttle_1::TIME,
    data::powerslide_turn_speed_1600_throttle_1::TIME,
    data::powerslide_turn_speed_1700_throttle_1::TIME,
    data::powerslide_turn_speed_1800_throttle_1::TIME,
    data::powerslide_turn_speed_1900_throttle_1::TIME,
    data::powerslide_turn_speed_2000_throttle_1::TIME,
    data::powerslide_turn_speed_2100_throttle_1::TIME,
    data::powerslide_turn_speed_2200_throttle_1::TIME,
    data::powerslide_turn_speed_2299_98_throttle_1::TIME,
];

lazy_static! {
    static ref POWERSLIDE_TURN_THROTTLE_0_CAR_LOC_2D: [&'static [Point2<f32>]; 24] = [
        *data::powerslide_turn_speed_0_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_100_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_200_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_300_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_400_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_500_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_600_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_700_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_800_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_900_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1000_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1100_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1200_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1300_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1400_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1500_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1600_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1700_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1800_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_1900_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_2000_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_2100_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_2200_throttle_0::CAR_LOC_2D,
        *data::powerslide_turn_speed_2299_98_throttle_0::CAR_LOC_2D,
    ];
    static ref POWERSLIDE_TURN_THROTTLE_1_CAR_LOC_2D: [&'static [Point2<f32>]; 24] = [
        *data::powerslide_turn_speed_0_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_100_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_200_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_300_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_400_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_500_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_600_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_700_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_800_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_900_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1000_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1100_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1200_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1300_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1400_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1500_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1600_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1700_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1800_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_1900_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_2000_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_2100_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_2200_throttle_1::CAR_LOC_2D,
        *data::powerslide_turn_speed_2299_98_throttle_1::CAR_LOC_2D,
    ];
}

const POWERSLIDE_TURN_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM: &[&[f32]] = &[
    data::powerslide_turn_speed_0_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_100_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_200_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_300_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_400_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_500_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_600_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_700_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_800_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_900_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1000_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1100_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1200_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1300_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1400_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1500_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1600_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1700_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1800_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1900_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2000_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2100_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2200_throttle_0::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2299_98_throttle_0::CAR_ROT_2D_ANGLE_CUM,
];

const POWERSLIDE_TURN_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM: &[&[f32]] = &[
    data::powerslide_turn_speed_0_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_100_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_200_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_300_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_400_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_500_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_600_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_700_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_800_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_900_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1000_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1100_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1200_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1300_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1400_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1500_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1600_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1700_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1800_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_1900_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2000_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2100_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2200_throttle_1::CAR_ROT_2D_ANGLE_CUM,
    data::powerslide_turn_speed_2299_98_throttle_1::CAR_ROT_2D_ANGLE_CUM,
];

lazy_static! {
    static ref POWERSLIDE_TURN_THROTTLE_0_CAR_VEL_2D: [&'static [Vector2<f32>]; 24] = [
        *data::powerslide_turn_speed_0_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_100_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_200_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_300_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_400_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_500_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_600_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_700_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_800_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_900_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1000_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1100_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1200_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1300_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1400_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1500_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1600_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1700_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1800_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_1900_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_2000_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_2100_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_2200_throttle_0::CAR_VEL_2D,
        *data::powerslide_turn_speed_2299_98_throttle_0::CAR_VEL_2D,
    ];
    static ref POWERSLIDE_TURN_THROTTLE_1_CAR_VEL_2D: [&'static [Vector2<f32>]; 24] = [
        *data::powerslide_turn_speed_0_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_100_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_200_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_300_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_400_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_500_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_600_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_700_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_800_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_900_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1000_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1100_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1200_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1300_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1400_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1500_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1600_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1700_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1800_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_1900_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_2000_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_2100_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_2200_throttle_1::CAR_VEL_2D,
        *data::powerslide_turn_speed_2299_98_throttle_1::CAR_VEL_2D,
    ];
}
