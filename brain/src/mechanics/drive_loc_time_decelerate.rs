use behavior::{Action, Behavior};
use common::prelude::*;
use eeg::{color, Drawable, EEG};
use mechanics::simple_steer_towards;
use nalgebra::Vector2;
use rlbot;
use simulate::Car1D;
use utils::my_car;

pub struct DriveLocTimeDecelerate {
    target_loc: Vector2<f32>,
    target_speed: f32,
    target_time: f32,
}

impl DriveLocTimeDecelerate {
    pub fn new(
        target_loc: Vector2<f32>,
        target_speed: f32,
        target_time: f32,
    ) -> DriveLocTimeDecelerate {
        DriveLocTimeDecelerate {
            target_loc,
            target_speed,
            target_time,
        }
    }
}

impl Behavior for DriveLocTimeDecelerate {
    fn name(&self) -> &str {
        stringify!(DriveLocTimeDecelerate)
    }

    fn execute(&mut self, packet: &rlbot::ffi::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);
        let distance = (me.Physics.loc().to_2d() - self.target_loc).norm();
        let time_remaining = self.target_time - packet.GameInfo.TimeSeconds;

        if time_remaining < 2.0 / 120.0 {
            return Action::Return;
        }

        eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
            me.Physics.rot(),
        ));
        eeg.draw(Drawable::print(
            format!("target_speed: {:.0}", self.target_speed),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));

        let mut result = rlbot::ffi::PlayerInput::default();
        result.Steer = simple_steer_towards(&me.Physics, self.target_loc);
        if estimate_approach(
            &me,
            distance,
            self.target_speed,
            time_remaining - 2.0 / 120.0,
        ) {
            result.Throttle = 1.0;
        }

        Action::Yield(result)
    }
}

/// Calculate a trajectory for traveling `distance` in `time` seconds which ends
/// at a final speed of `target_speed`.
///
/// This basically works by running time forwards from the car's position, and
/// also running time backwards from the target location, and choosing an
/// intersection of the two paths where the distance traveled is close to the
/// target distance.
fn estimate_approach(
    car: &rlbot::ffi::PlayerInfo,
    distance: f32,
    target_speed: f32,
    time: f32,
) -> bool {
    const DT: f32 = 1.0 / 60.0;

    let mut t_accel = 1.5 / 120.0;
    let mut sim_accel = Car1D::new(car.Physics.vel().norm());
    let mut accel = Vec::new();

    if t_accel >= time {
        return true; // Time is up, there's nothing more we can do!
    }

    while t_accel < time {
        t_accel += DT;
        sim_accel.step(DT, 1.0, false);

        accel.push(SS {
            time: t_accel,
            speed: sim_accel.speed(),
            distance: sim_accel.distance_traveled(),
        });
    }

    let mut t_coast = 1.5 / 120.0;
    let mut sim_coast = Car1D::new(target_speed);
    let mut coast = Vec::new();

    while t_coast < time {
        t_coast += DT;
        sim_coast.step_rev(DT, 0.0, false);

        coast.push(SS {
            time: t_coast,
            speed: sim_coast.speed(),
            distance: sim_coast.distance_traveled(),
        });
    }

    let (cross_accel, cross_coast) = accel
        .iter()
        .zip(coast.iter().rev())
        .min_by(|&(a1, c1), &(a2, c2)| {
            ((a1.speed - c1.speed).abs())
                .partial_cmp(&(a2.speed - c2.speed).abs())
                .unwrap()
        })
        .unwrap();

    let result = if cross_accel.distance + cross_coast.distance >= distance {
        false
    } else if cross_accel.time == accel[0].time {
        false
    } else {
        true
    };
    // println!(
    //     "{}   {:.0}   {:.0}   {:.0}   {:.0}   {:?}   {:?}",
    //     result,
    //     distance,
    //     cross_accel.distance + cross_coast.distance,
    //     accel[0].speed,
    //     coast[0].speed,
    //     cross_accel,
    //     cross_coast,
    // );
    result
}

#[derive(Debug)]
struct SS {
    time: f32,
    speed: f32,
    distance: f32,
}

#[cfg(test)]
mod integration_tests {
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use mechanics::DriveLocTimeDecelerate;
    use nalgebra::{Vector2, Vector3};

    #[test]
    fn verify_arrival_time() {
        let cases = [
            (300.0, 0.0, 1000.0, 300.0),
            (300.0, 0.0, 1000.0, 600.0),
            (300.0, 0.0, 1000.0, 100.0),
            // This last test does a pretty interesting correction right at the
            // end and it seems like it would be the ideal case to study if I
            // wanted to improve this mechanic.
            (1000.0, 0.0, 1000.0, 400.0),
        ];
        for &(initial_speed, target_x, target_y, target_speed) in cases.iter() {
            let target_loc = Vector2::new(target_x, target_y);
            let test = TestRunner::start2(
                TestScenario {
                    ball_loc: Vector3::new(2000.0, 0.0, 0.0),
                    car_vel: Vector3::new(0.0, initial_speed, 0.0),
                    ..Default::default()
                },
                move |p| {
                    DriveLocTimeDecelerate::new(
                        target_loc,
                        target_speed,
                        p.GameInfo.TimeSeconds + 2.0,
                    )
                },
            );

            test.sleep_millis(2000);

            let packet = test.sniff_packet();
            let discrepancy = (packet.GameCars[0].Physics.loc().to_2d() - target_loc).norm();
            println!("target loc: {:.?}", target_loc);
            println!("car loc: {:.?}", packet.GameCars[0].Physics.loc());
            println!("discrepancy: {:.0}", discrepancy);
            println!("target speed: {:.0}", target_speed);
            let car_speed = packet.GameCars[0].Physics.vel().norm();
            println!("car speed: {:.0}", car_speed);
            assert!(discrepancy.abs() < 50.0);
            assert!((target_speed - car_speed).abs() < 100.0);
        }
    }
}
