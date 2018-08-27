use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::simple_steer_towards;
use nalgebra::Vector3;
use rlbot;
use simulate::{rl, Car1D};
use utils::{my_car, ExtendPhysics};

pub struct DriveLocTimeDecelerate {
    target_loc: Vector3<f32>,
    target_speed: f32,
    target_time: f32,
}

impl DriveLocTimeDecelerate {
    pub fn new(
        target_loc: Vector3<f32>,
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
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);
        let distance = (me.Physics.loc() - self.target_loc).norm();
        let time_remaining = self.target_time - packet.GameInfo.TimeSeconds;

        eeg.draw(Drawable::print("drive_loc_time_decelerate", color::YELLOW));
        eeg.draw(Drawable::GhostCar(self.target_loc, me.Physics.rot()));
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

        let mut result = rlbot::PlayerInput::default();
        result.Steer = simple_steer_towards(&me.Physics, self.target_loc);
        if !estimate_approach(
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

/// Starting at `origin`, if we coast for `time` seconds, will
/// we have traveled `distance`?
fn estimate_approach(car: &rlbot::PlayerInfo, distance: f32, target_speed: f32, time: f32) -> bool {
    const DT: f32 = 1.0 / 60.0;

    let mut t_accel = 1.5 / 120.0;
    let mut sim_accel = Car1D::new(car.Physics.vel().norm());
    let mut accel = Vec::new();

    while t_accel < time {
        t_accel += DT;
        sim_accel.step(DT, 1.0, false);

        let mut t_coast = t_accel;
        let mut sim_coast = Car1D::new(sim_accel.speed());
        while t_coast < time {
            t_coast += DT;
            sim_coast.step(DT, 0.0, false);
        }

        accel.push((
            t_accel,
            sim_accel.speed(),
            sim_accel.distance_traveled() + sim_coast.distance_traveled(),
        ))
    }

    let x = accel.iter();
    for x in x {
        println!("{:?}", x)
    }
    unimplemented!();
    //        .any(|(at, d)| d >= distance)
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use mechanics::DriveLocTimeDecelerate;
    use nalgebra::Vector3;
    use utils::{ExtendPhysics, ExtendVector3};

    #[test]
    fn verify_arrival_time() {
        let cases = [
            (300.0, 0.0, 1000.0, 600.0),
            (300.0, 0.0, 1000.0, 100.0),
            (1000.0, 0.0, 1000.0, 400.0),
        ];
        for &(initial_speed, target_x, target_y, target_speed) in cases.iter() {
            let target_loc = Vector3::new(target_x, target_y, 0.0);
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
            let diff = (packet.GameCars[0].Physics.loc() - target_loc)
                .to_2d()
                .norm();
            println!("target loc: {:.?}", target_loc);
            println!("car loc: {:.?}", packet.GameCars[0].Physics.loc());
            println!("diff: {:.0}", diff);
            assert!(diff.abs() < 20.0);
        }
    }
}
