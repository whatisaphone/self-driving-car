use behavior::{Action, Behavior};
use common::{prelude::*, rl};
use eeg::{color, Drawable};
use maneuvers::{drive_towards, BounceShot, GetToFlatGround};
use mechanics::simple_yaw_diff;
use predict::naive_ground_intercept;
use rlbot;
use rules::SameBallTrajectory;
use simulate::{Car1D, CarAerial60Deg};
use std::f32::consts::PI;
use strategy::Context;

const Z_FUDGE: f32 = 75.0;

pub struct JumpShot {
    same_ball_trajectory: SameBallTrajectory,
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
            same_ball_trajectory: SameBallTrajectory::new(),
            phase: Phase::Ground,
        }
    }
}

impl Behavior for JumpShot {
    fn name(&self) -> &str {
        stringify!(JumpShot)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        match self.phase {
            Phase::Ground => {
                return_some!(self.same_ball_trajectory.execute(ctx));
                self.ground(ctx)
            }
            Phase::Air {
                start_time,
                duration,
            } => self.air(ctx, start_time, duration),
            Phase::Dodge { start_time } => self.dodge(ctx, start_time),
            Phase::Finished => Action::Return,
        }
    }
}

impl JumpShot {
    fn ground(&mut self, ctx: &mut Context) -> Action {
        if !GetToFlatGround::on_flat_ground(ctx.packet) {
            return Action::Abort;
        }

        let me = ctx.me();

        let intercept = naive_ground_intercept(
            ctx.scenario.ball_prediction().iter(),
            me.Physics.locp(),
            me.Physics.vel(),
            me.Boost as f32,
            |ball| {
                let air_time = CarAerial60Deg::cost(ball.loc.z - Z_FUDGE).time;
                if ball.t < air_time {
                    return false;
                }

                ball.loc.z < Self::MAX_BALL_Z
            },
        );

        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[JumpShot] no good intercept");
            return Action::Abort;
        });

        let aim_loc = BounceShot::aim_loc(
            ctx.game.enemy_goal(),
            me.Physics.locp().to_2d(),
            intercept.ball_loc.to_2d(),
        );
        let target_loc = BounceShot::rough_shooting_spot(&intercept, aim_loc);
        let yaw_diff = simple_yaw_diff(&me.Physics, target_loc.coords);
        let distance = (me.Physics.locp().to_2d() - target_loc).norm();
        let air_time = CarAerial60Deg::cost(intercept.ball_loc.z - Z_FUDGE).time;
        let available_ground_time = intercept.time - air_time;

        ctx.eeg
            .draw(Drawable::print(stringify!(Phase::Ground), color::GREEN));
        ctx.eeg.draw(Drawable::GhostBall(intercept.ball_loc.coords));
        ctx.eeg.draw(Drawable::Crosshair(aim_loc.coords));
        ctx.eeg.draw(Drawable::GhostCar(
            target_loc.to_3d(intercept.ball_loc.z).coords,
            me.Physics.rot(),
        ));
        ctx.eeg.draw(Drawable::print(
            format!("yaw_diff: {:.0}°", yaw_diff.to_degrees()),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("air_time: {:.2}", air_time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("avail_ground_time: {:.2}", available_ground_time),
            color::GREEN,
        ));

        if available_ground_time < 2.0 / 120.0 && yaw_diff.abs() < 5.0_f32.to_radians() {
            ctx.eeg.log("[JumpShot] Air");
            self.phase = Phase::Air {
                start_time: ctx.packet.GameInfo.TimeSeconds,
                duration: air_time,
            };
            return self.execute2(ctx);
        }

        let yaw_diff = simple_yaw_diff(&me.Physics, target_loc.coords);
        let too_fast = estimate_approach(
            me.Physics.vel().norm(),
            distance,
            available_ground_time - 2.0 / 120.0,
            air_time,
        );

        let mut result = drive_towards(ctx.packet, ctx.eeg, target_loc.coords);
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

    fn air(&mut self, ctx: &mut Context, start_time: f32, duration: f32) -> Action {
        let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;
        let time_remaining = duration - elapsed;

        ctx.eeg
            .draw(Drawable::print(stringify!(Phase::Air), color::GREEN));
        ctx.eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));

        if time_remaining <= 0.0 {
            ctx.eeg.log("[JumpShot] Dodge");
            self.phase = Phase::Dodge {
                start_time: ctx.packet.GameInfo.TimeSeconds,
            };
            return self.execute2(ctx);
        }

        Action::Yield(rlbot::ffi::PlayerInput {
            Jump: time_remaining > 0.05,
            ..Default::default()
        })
    }

    fn dodge(&mut self, ctx: &mut Context, start_time: f32) -> Action {
        ctx.eeg
            .draw(Drawable::print(stringify!(Phase::Dodge), color::GREEN));

        let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;
        if elapsed >= 0.05 {
            self.phase = Phase::Finished;
            return self.execute2(ctx);
        }

        let me = ctx.me();
        let angle = simple_yaw_diff(&me.Physics, ctx.packet.GameBall.Physics.loc().to_2d());

        Action::Yield(rlbot::ffi::PlayerInput {
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
    use behavior::Repeat;
    use common::prelude::*;
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

    #[test]
    fn correct_mispredicted_bounce() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-3350.3652, 2287.6494, 537.7215),
            ball_vel: Vector3::new(583.83765, 331.18698, -33.879772),
            car_loc: Vector3::new(-3420.127, 75.629135, 17.02),
            car_rot: Rotation3::from_unreal_angles(-0.009491506, 0.52864814, -0.0001917476),
            car_vel: Vector3::new(1083.5627, 572.17487, 8.241),
            ..Default::default()
        });
        test.set_behavior(Repeat::new(JumpShot::new));

        test.sleep_millis(3000);
        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "[JumpShot] Air"));
        });
        assert!(test.has_scored());
    }
}
