use common::{
    ext::{ExtendUnitVector2, ExtendVector2},
    physics::{car_forward_axis_2d, CAR_LOCAL_FORWARD_AXIS_2D},
};
use nalgebra::{Point2, Unit, Vector2};
use tables;

pub struct CarPowerslideTurn;

impl CarPowerslideTurn {
    #[used]
    fn evaluate(
        start_loc: Point2<f32>,
        start_vel: Vector2<f32>,
        throttle: f32,
        target_dir: Unit<Vector2<f32>>,
    ) -> Option<(f32, Point2<f32>)> {
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
        let (reference_time, mut reference_offset) = some_or_else!(reference, {
            return None;
        });
        reference_offset.x *= steer;
        let reference_offset = reference_offset;

        // Transform the reference units back into input units.
        let offset = transform.inverse() * reference_offset;
        let loc = start_loc + offset;
        Some((reference_time, loc))
    }

    /// Assume a car traveling at `start_speed`, which immediately hits the
    /// handbrake, turns right (steer 1.0), and sets throttle to `throttle`.
    /// Then wait for the car to rotate by `target_rot_by` radians. How long
    /// will it take, and how far will the car have traveled?
    fn reference_evaluate(
        start_speed: f32,
        throttle: f32,
        target_rot_by: f32,
    ) -> Option<(f32, Vector2<f32>)> {
        let speed_index = start_speed as usize / 100;
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

        // TODO: interpolate between `lower` and `upper`
        let time = lower.time[index] - lower.time[0];
        let loc = lower.loc_2d[index] - lower.loc_2d[0];
        Some((time, loc))
    }
}

struct TableSet<'a> {
    time: &'a [f32],
    loc_2d: &'a [Point2<f32>],
    rot_2d_angle_cum: &'a [f32],
}

impl<'a> TableSet<'a> {
    fn get(throttle: f32, speed_index: usize) -> Self {
        if throttle == 0.0 {
            Self {
                time: POWERSLIDE_TURN_THROTTLE_0_TIME[speed_index],
                loc_2d: POWERSLIDE_TURN_THROTTLE_0_CAR_LOC_2D[speed_index],
                rot_2d_angle_cum: POWERSLIDE_TURN_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM[speed_index],
            }
        } else if throttle == 1.0 {
            Self {
                time: POWERSLIDE_TURN_THROTTLE_1_TIME[speed_index],
                loc_2d: POWERSLIDE_TURN_THROTTLE_1_CAR_LOC_2D[speed_index],
                rot_2d_angle_cum: POWERSLIDE_TURN_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM[speed_index],
            }
        } else {
            panic!("throttle not quantized")
        }
    }
}

const POWERSLIDE_TURN_THROTTLE_0_TIME: &[&[f32]] = &[
    tables::POWERSLIDE_TURN_SPEED_0_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_100_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_200_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_300_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_400_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_500_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_600_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_700_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_800_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_900_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1000_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1100_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1200_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1300_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1400_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1500_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1600_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1700_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1800_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_1900_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_2000_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_2100_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_2200_THROTTLE_0_TIME,
    tables::POWERSLIDE_TURN_SPEED_2299_98_THROTTLE_0_TIME,
];

const POWERSLIDE_TURN_THROTTLE_1_TIME: &[&[f32]] = &[
    tables::POWERSLIDE_TURN_SPEED_0_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_100_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_200_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_300_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_400_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_500_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_600_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_700_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_800_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_900_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1000_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1100_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1200_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1300_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1400_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1500_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1600_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1700_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1800_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_1900_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_2000_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_2100_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_2200_THROTTLE_1_TIME,
    tables::POWERSLIDE_TURN_SPEED_2299_98_THROTTLE_1_TIME,
];

const POWERSLIDE_TURN_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM: &[&[f32]] = &[
    tables::POWERSLIDE_TURN_SPEED_0_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_100_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_200_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_300_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_400_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_500_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_600_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_700_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_800_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_900_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1000_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1100_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1200_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1300_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1400_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1500_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1600_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1700_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1800_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1900_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2000_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2100_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2200_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2299_98_THROTTLE_0_CAR_ROT_2D_ANGLE_CUM,
];

const POWERSLIDE_TURN_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM: &[&[f32]] = &[
    tables::POWERSLIDE_TURN_SPEED_0_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_100_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_200_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_300_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_400_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_500_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_600_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_700_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_800_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_900_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1000_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1100_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1200_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1300_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1400_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1500_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1600_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1700_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1800_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_1900_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2000_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2100_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2200_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
    tables::POWERSLIDE_TURN_SPEED_2299_98_THROTTLE_1_CAR_ROT_2D_ANGLE_CUM,
];

lazy_static! {
    static ref POWERSLIDE_TURN_THROTTLE_0_CAR_LOC_2D: [&'static [Point2<f32>]; 24] = [
        &*tables::POWERSLIDE_TURN_SPEED_0_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_100_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_200_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_300_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_400_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_500_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_600_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_700_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_800_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_900_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1000_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1100_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1200_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1300_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1400_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1500_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1600_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1700_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1800_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1900_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2000_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2100_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2200_THROTTLE_0_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2299_98_THROTTLE_0_CAR_LOC_2D,
    ];
    static ref POWERSLIDE_TURN_THROTTLE_1_CAR_LOC_2D: [&'static [Point2<f32>]; 24] = [
        &*tables::POWERSLIDE_TURN_SPEED_0_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_100_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_200_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_300_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_400_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_500_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_600_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_700_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_800_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_900_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1000_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1100_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1200_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1300_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1400_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1500_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1600_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1700_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1800_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_1900_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2000_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2100_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2200_THROTTLE_1_CAR_LOC_2D,
        &*tables::POWERSLIDE_TURN_SPEED_2299_98_THROTTLE_1_CAR_LOC_2D,
    ];
}
