use behavior::{Action, Behavior};
use common::prelude::*;
use eeg::{color, Drawable};
use maneuvers::BlitzToLocation;
use mechanics::simple_steer_towards;
use nalgebra::Vector2;
use plan::drive::rough_time_drive_to_loc;
use rlbot;
use rules::SameBallTrajectory;
use simulate::rl;
use strategy::Context;
use utils::{my_goal_center_2d, ExtendF32, ExtendVector2};

pub struct PanicDefense {
    use_boost: bool,
    same_ball_trajectory: SameBallTrajectory,
    phase: Phase,
}

enum Phase {
    Start,
    Rush {
        aim_hint: Vector2<f32>,
        child: BlitzToLocation,
    },
    Turn {
        aim_hint: Vector2<f32>,
        target_yaw: f32,
        start_time: f32,
    },
    Finished,
}

impl PanicDefense {
    pub fn new() -> Self {
        Self {
            use_boost: true,
            same_ball_trajectory: SameBallTrajectory::new(),
            phase: Phase::Start,
        }
    }

    pub fn use_boost(self, use_boost: bool) -> Self {
        Self { use_boost, ..self }
    }
}

impl Behavior for PanicDefense {
    fn name(&self) -> &str {
        stringify!(PanicDefense)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        return_some!(self.same_ball_trajectory.execute(ctx));

        if let Some(phase) = self.next_phase(ctx) {
            self.phase = phase;
        }

        let me = ctx.me();
        let target_loc = my_goal_center_2d();
        ctx.eeg.draw(Drawable::print(
            format!("use_boost: {}", self.use_boost),
            color::GREEN,
        ));
        ctx.eeg
            .draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));

        match self.phase {
            Phase::Start => unreachable!(),
            Phase::Rush { ref mut child, .. } => {
                ctx.eeg.draw(Drawable::print("Rush", color::GREEN));
                child.execute2(ctx)
            }
            Phase::Turn { aim_hint, .. } => {
                ctx.eeg.draw(Drawable::print("Turn", color::GREEN));
                Action::Yield(rlbot::ffi::PlayerInput {
                    Throttle: 1.0,
                    Steer: simple_steer_towards(&me.Physics, aim_hint),
                    Handbrake: true,
                    ..Default::default()
                })
            }
            Phase::Finished => Action::Return,
        }
    }
}

impl PanicDefense {
    fn blitz_loc(aim_loc: Vector2<f32>) -> Vector2<f32> {
        Vector2::new(800.0 * -aim_loc.x.signum(), my_goal_center_2d().y)
    }

    fn next_phase(&mut self, ctx: &mut Context) -> Option<Phase> {
        let me = ctx.me();

        if let Phase::Start = self.phase {
            let aim_hint = calc_aim_hint(ctx);
            return Some(Phase::Rush {
                aim_hint,
                child: BlitzToLocation::new(Self::blitz_loc(aim_hint)),
            });
        }

        match self.phase {
            Phase::Rush { .. } | Phase::Turn { .. } => {
                if me.Physics.loc().y <= -5000.0 {
                    return Some(Phase::Finished);
                }
            }
            _ => {}
        }

        if let Phase::Turn {
            start_time,
            target_yaw,
            ..
        } = self.phase
        {
            let theta = (me.Physics.rot().yaw() - target_yaw).normalize_angle();
            if theta.abs() <= 15.0_f32.to_radians() {
                ctx.eeg.log("done, facing the right way");
                return Some(Phase::Finished);
            }
            // Fail-safe
            let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;
            if elapsed >= 0.75 {
                ctx.eeg.log("done, time elapsed failsafe");
                return Some(Phase::Finished);
            }
        }

        if let Phase::Rush { aim_hint, .. } = self.phase {
            let cutoff = -rl::FIELD_MAX_Y - me.Physics.vel().y * 0.75;
            ctx.eeg.draw(Drawable::print(
                format!("cutoff_distance: {:.0}", me.Physics.loc().y - cutoff),
                color::GREEN,
            ));
            if me.Physics.loc().y <= cutoff {
                let target_yaw = my_goal_center_2d().angle_to(aim_hint);
                return Some(Phase::Turn {
                    aim_hint: calc_aim_hint(ctx),
                    start_time: ctx.packet.GameInfo.TimeSeconds,
                    target_yaw,
                });
            }
        }

        None
    }
}

fn calc_aim_hint(ctx: &mut Context) -> Vector2<f32> {
    // When we reach goal, which half of the field will the ball be on?
    let time = rough_time_drive_to_loc(ctx.me(), my_goal_center_2d());
    let sim_ball = ctx.scenario.ball_prediction().at_time(time);
    let sim_ball_loc = match sim_ball {
        Some(b) => b.loc.coords,
        None => ctx.packet.GameBall.Physics.loc(),
    };
    Vector2::new(
        sim_ball_loc.x.signum() * rl::FIELD_MAX_X,
        my_goal_center_2d().y,
    )
}

#[cfg(test)]
mod integration_tests {
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::PanicDefense;
    use nalgebra::{Rotation3, Vector3};
    use std::f32::consts::PI;

    #[test]
    fn panic_defense() {
        let test = TestRunner::start0(TestScenario {
            car_loc: Vector3::new(500.0, -1000.0, 17.01),
            car_rot: Rotation3::from_unreal_angles(0.0, -PI / 2.0, 0.0),
            car_vel: Vector3::new(0.0, 0.0, 0.0),
            ..Default::default()
        });
        test.set_behavior(PanicDefense::new());

        test.sleep_millis(4000);

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameCars[0].Physics.vel());
        assert!(packet.GameCars[0].Physics.loc().y < -4500.0);
        assert!(packet.GameCars[0].Physics.vel().norm() < 100.0);
    }
}
