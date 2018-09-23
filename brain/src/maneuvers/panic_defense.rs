use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use eeg::{color, Drawable, EEG};
use maneuvers::BlitzToLocation;
use mechanics::simple_steer_towards;
use nalgebra::Vector2;
use plan::{ball::predict_ball, drive::rough_time_drive_to_loc};
use rlbot;
use simulate::rl;
use utils::{my_car, my_goal_center_2d, ExtendF32, ExtendPhysics, ExtendVector2};

pub struct PanicDefense {
    use_boost: bool,
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
            phase: Phase::Start,
        }
    }

    pub fn use_boost(self, use_boost: bool) -> Self {
        Self { use_boost, ..self }
    }
}

impl Behavior for PanicDefense {
    fn name(&self) -> &'static str {
        stringify!(PanicDefense)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if let Some(phase) = self.next_phase(packet, eeg) {
            self.phase = phase;
        }

        let me = my_car(packet);
        let target_loc = my_goal_center_2d();
        eeg.draw(Drawable::print(
            format!("use_boost: {}", self.use_boost),
            color::GREEN,
        ));
        eeg.draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));

        match self.phase {
            Phase::Start => unreachable!(),
            Phase::Rush { ref mut child, .. } => {
                eeg.draw(Drawable::print("Rush", color::GREEN));
                child.execute(packet, eeg)
            }
            Phase::Turn { aim_hint, .. } => {
                eeg.draw(Drawable::print("Turn", color::GREEN));
                Action::Yield(rlbot::PlayerInput {
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

    fn next_phase(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Option<Phase> {
        let me = my_car(packet);

        if let Phase::Start = self.phase {
            let aim_hint = calc_aim_hint(&packet.GameBall, me);
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
                eeg.log("done, facing the right way");
                return Some(Phase::Finished);
            }
            // Fail-safe
            let elapsed = packet.GameInfo.TimeSeconds - start_time;
            if elapsed >= 0.75 {
                eeg.log("done, time elapsed failsafe");
                return Some(Phase::Finished);
            }
        }

        if let Phase::Rush { aim_hint, .. } = self.phase {
            let cutoff = -rl::FIELD_MAX_Y - me.Physics.vel().y * 0.75;
            eeg.draw(Drawable::print(
                format!("cutoff_distance: {:.0}", me.Physics.loc().y - cutoff),
                color::GREEN,
            ));
            if me.Physics.loc().y <= cutoff {
                let target_yaw = my_goal_center_2d().angle_to(aim_hint);
                return Some(Phase::Turn {
                    aim_hint: calc_aim_hint(&packet.GameBall, me),
                    start_time: packet.GameInfo.TimeSeconds,
                    target_yaw,
                });
            }
        }

        None
    }
}

fn calc_aim_hint(ball: &rlbot::BallInfo, car: &rlbot::PlayerInfo) -> Vector2<f32> {
    // When we reach goal, which half of the field will the ball be on?
    let time = rough_time_drive_to_loc(car, my_goal_center_2d());
    let sim_ball = predict_ball(ball, |t, _, _| t >= time);
    let sim_ball_loc = match sim_ball {
        Some(b) => b.loc(),
        None => ball.Physics.loc(),
    };
    Vector2::new(
        sim_ball_loc.x.signum() * rl::FIELD_MAX_X,
        my_goal_center_2d().y,
    )
}

#[cfg(test)]
mod integration_tests {
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::PanicDefense;
    use nalgebra::{Rotation3, Vector2, Vector3};
    use std::f32::consts::PI;
    use utils::ExtendPhysics;

    #[test]
    fn panic_defense() {
        let test = TestRunner::start(
            PanicDefense::new(),
            TestScenario {
                car_loc: Vector3::new(500.0, -1000.0, 17.01),
                car_rot: Rotation3::from_unreal_angles(0.0, -PI / 2.0, 0.0),
                car_vel: Vector3::new(0.0, 0.0, 0.0),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameCars[0].Physics.vel());
        assert!(packet.GameCars[0].Physics.loc().y < -4500.0);
        assert!(packet.GameCars[0].Physics.vel().norm() < 100.0);
    }
}
