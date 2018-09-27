use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::{drive_towards, BounceShot, GetToFlatGround};
use mechanics::simple_yaw_diff;
use predict::estimate_intercept_car_ball_3;
use rlbot;
use simulate::{rl, Car1D, CarAerial60Deg};
use std::f32::consts::PI;
use utils::{one_v_one, ExtendPhysics, ExtendVector2, ExtendVector3};

const Z_FUDGE: f32 = 75.0;

pub struct JumpShot {
    phase: Phase,
}

enum Phase {
    Ground,
    Air { start_time: f32, duration: f32 },
    Dodge { start_time: f32 },
    Finished,
}

impl JumpShot {
    pub const MAX_BALL_Z: f32 = 320.0; // sloppy number

    pub fn new() -> JumpShot {
        JumpShot {
            phase: Phase::Ground,
        }
    }
}

impl Behavior for JumpShot {
    fn name(&self) -> &'static str {
        stringify!(JumpShot)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        match self.phase {
            Phase::Ground => self.ground(packet, eeg),
            Phase::Air {
                start_time,
                duration,
            } => self.air(packet, eeg, start_time, duration),
            Phase::Dodge { start_time } => self.dodge(packet, eeg, start_time),
            Phase::Finished => Action::Return,
        }
    }
}

impl JumpShot {
    fn ground(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if !GetToFlatGround::on_flat_ground(packet) {
            // TODO: this is not how this works…
            return Action::call(GetToFlatGround::new());
        }

        let (me, _enemy) = one_v_one(packet);

        let intercept = estimate_intercept_car_ball_3(&me, &packet.GameBall, |t, &loc, _vel| {
            let air_time = CarAerial60Deg::cost(loc.z - Z_FUDGE).time;
            if t < air_time {
                return false;
            }

            loc.z < Self::MAX_BALL_Z
        });

        let intercept = match intercept {
            Some(i) => i,
            None => {
                eeg.log("[JumpShot] intercept not found; aborting");
                return Action::Abort;
            }
        };

        let aim_loc = BounceShot::aim_loc(me.Physics.loc().to_2d(), intercept.ball_loc.to_2d());
        let target_loc = BounceShot::rough_shooting_spot(&intercept, aim_loc);
        let yaw_diff = simple_yaw_diff(&me.Physics, target_loc);
        let distance = (me.Physics.loc().to_2d() - target_loc).norm();
        let air_time = CarAerial60Deg::cost(intercept.ball_loc.z - Z_FUDGE).time;
        let available_ground_time = intercept.time - air_time;

        eeg.draw(Drawable::print(stringify!(Phase::Ground), color::GREEN));
        eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        eeg.draw(Drawable::Crosshair(aim_loc));
        eeg.draw(Drawable::GhostCar(
            target_loc.to_3d(intercept.ball_loc.z),
            me.Physics.rot(),
        ));
        eeg.draw(Drawable::print(
            format!("yaw_diff: {:.0}°", yaw_diff.to_degrees()),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("air_time: {:.2}", air_time),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("avail_ground_time: {:.2}", available_ground_time),
            color::GREEN,
        ));

        if available_ground_time < 2.0 / 120.0 && yaw_diff.abs() < 5.0_f32.to_radians() {
            self.phase = Phase::Air {
                start_time: packet.GameInfo.TimeSeconds,
                duration: air_time,
            };
            return self.execute(packet, eeg);
        }

        let yaw_diff = simple_yaw_diff(&me.Physics, target_loc);
        let too_fast = estimate_approach(
            me.Physics.vel().norm(),
            distance,
            available_ground_time - 2.0 / 120.0,
            air_time,
        );

        let mut result = drive_towards(packet, eeg, target_loc);
        if too_fast {
            result.Throttle = 0.0;
        } else {
            if me.OnGround
                && yaw_diff.abs() < PI / 4.0
                && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED
                && me.Boost > 0
            {
                result.Boost = true;
            }
        }

        Action::Yield(result)
    }

    fn air(
        &mut self,
        packet: &rlbot::LiveDataPacket,
        eeg: &mut EEG,
        start_time: f32,
        duration: f32,
    ) -> Action {
        let elapsed = packet.GameInfo.TimeSeconds - start_time;
        let time_remaining = duration - elapsed;

        eeg.draw(Drawable::print(stringify!(Phase::Air), color::GREEN));
        eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));

        if time_remaining <= 0.0 {
            self.phase = Phase::Dodge {
                start_time: packet.GameInfo.TimeSeconds,
            };
            return self.execute(packet, eeg);
        }

        Action::Yield(rlbot::PlayerInput {
            Jump: time_remaining > 0.05,
            ..Default::default()
        })
    }

    fn dodge(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG, start_time: f32) -> Action {
        eeg.draw(Drawable::print(stringify!(Phase::Dodge), color::GREEN));

        let elapsed = packet.GameInfo.TimeSeconds - start_time;
        if elapsed >= 0.05 {
            self.phase = Phase::Finished;
            return self.execute(packet, eeg);
        }

        let (me, _enemy) = one_v_one(packet);
        let angle = simple_yaw_diff(&me.Physics, packet.GameBall.Physics.loc().to_2d());

        Action::Yield(rlbot::PlayerInput {
            Pitch: -angle.cos(),
            Yaw: angle.sin(),
            Jump: true,
            ..Default::default()
        })
    }
}

/// Will we reach the target if we go pedal to the metal?
fn estimate_approach(
    initial_speed: f32,
    target_dist_2d: f32,
    ground_time: f32,
    air_time: f32,
) -> bool {
    const DT: f32 = 1.0 / 60.0;

    let mut t = 2.0 / 120.0;

    let mut car = Car1D::new(initial_speed);
    while t < ground_time {
        car.step(DT, 1.0, true);
        t += DT;
    }

    let mut distance = car.distance_traveled();

    // During the jump we can't noticeably accelerate, so simulate constant
    // velocity.
    while t < ground_time + air_time {
        t += DT;
        distance += car.speed() * DT;
    }

    distance >= target_dist_2d
}

#[cfg(test)]
mod integration_tests {
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::jump_shot::JumpShot;
    use nalgebra::{Rotation3, Vector3};

    #[test]
    fn far_corner_falling() {
        let test = TestRunner::start(
            JumpShot::new(),
            TestScenario {
                ball_loc: Vector3::new(2790.6147, 4609.1733, 101.71101),
                ball_vel: Vector3::new(-1887.2709, 1104.1567, -66.64522),
                car_loc: Vector3::new(3066.908, 4276.6646, 20.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 3.43, 0.0),
                car_vel: Vector3::new(-1351.3594, 345.7535, 108.78551),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn sideways_jump() {
        let test = TestRunner::start(
            JumpShot::new(),
            TestScenario {
                ball_loc: Vector3::new(3882.8804, 1246.1459, 145.22563),
                ball_vel: Vector3::new(-48.063526, 172.48882, -818.2703),
                car_loc: Vector3::new(3656.8044, -12.230183, 16.709408),
                car_rot: Rotation3::from_unreal_angles(-0.0069029136, 2.6237783, -0.002396845),
                car_vel: Vector3::new(-855.45123, 459.4182, 14.042989),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        unimplemented!();
    }
}
