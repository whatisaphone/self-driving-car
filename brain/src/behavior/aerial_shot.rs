use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::{simple_steer_towards, simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use predict::estimate_intercept_car_ball_2;
use rlbot;
use simulate::rl;
use simulate::CarAerial60Deg;
use utils::{enemy_goal_center, my_car, one_v_one, ExtendPhysics, ExtendVector3};

pub struct AerialShot {
    phase: Phase,
    min_distance: Option<f32>,
}

#[derive(Debug)]
enum Phase {
    Ground,
    Air { start_time: f32 },
    Shoot,
}

impl AerialShot {
    pub fn new() -> AerialShot {
        AerialShot {
            phase: Phase::Ground,
            min_distance: None,
        }
    }
}

impl Behavior for AerialShot {
    fn name(&self) -> &'static str {
        "AerialShot"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let ball_dist = (me.Physics.loc() - packet.GameBall.Physics.loc()).norm();

        // If the ball has moved further away, assume we hit it (or something
        // unexpected has happened).
        match self.min_distance {
            Some(md) if ball_dist >= md + 250.0 => return Action::Return,
            _ => self.min_distance = Some(ball_dist),
        }

        eeg.draw(Drawable::print(format!("{:?}", self.phase), color::GREEN));

        match self.phase {
            Phase::Ground => ground(packet, eeg).unwrap_or_else(|| {
                let start_time = packet.GameInfo.TimeSeconds;
                self.phase = Phase::Air { start_time };
                self.execute(packet, eeg)
            }),
            Phase::Air { start_time } => air(packet, eeg, start_time).unwrap_or_else(|| {
                self.phase = Phase::Shoot;
                self.execute(packet, eeg)
            }),
            Phase::Shoot => unimplemented!(),
        }
    }
}

fn ground(packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Option<Action> {
    let me = my_car(packet);

    if !GetToFlatGround::on_flat_ground(packet) {
        warn!("Not on flat ground");
        return Some(Action::Return);
    }

    let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |t, loc, vel| {
        let max_comfortable_z = rl::CROSSBAR_Z + enemy_goal_center().y - loc.y;
        if loc.z >= max_comfortable_z {
            return false;
        }
        let cost = CarAerial60Deg::cost(loc.z);
        cost.time < t
    });

    eeg.draw(Drawable::print(
        format!("intercept_time: {:.2}", intercept.time),
        color::GREEN,
    ));
    //    eeg.draw(Drawable::print(
    //        format!("aerial_time: {:.2}", cost.time),
    //        color::GREEN,
    //    ));
    let yd = simple_yaw_diff(&me.Physics, intercept.ball_loc.to_2d()).abs();
    println!(
        "{:.2} {:.0} {:.2}",
        intercept.time, intercept.ball_loc.z, yd,
    );

    if yd < 10.0_f32.to_radians() {
        return None; // Advance to next phase.
    }

    // TODO: this is not how this works
    let mut child = GroundAccelToLoc::new(
        intercept.ball_loc.to_2d(),
        packet.GameInfo.TimeSeconds + intercept.time,
    );
    Some(child.execute(packet, eeg))
}

fn air(packet: &rlbot::LiveDataPacket, eeg: &mut EEG, start_time: f32) -> Option<Action> {
    let phase_time = packet.GameInfo.TimeSeconds - start_time;
    let target_pitch = 60.0_f32.to_radians();
    let pitch = (target_pitch - packet.GameCars[0].Physics.Rotation.Pitch) / 2.0;
    let input = rlbot::PlayerInput {
        Pitch: pitch.max(-1.0).min(1.0),
        Jump: true,
        Boost: phase_time >= 0.25,
        ..Default::default()
    };
    Some(Action::Yield(input))
}

#[cfg(test)]
mod integration_tests {
    use behavior::aerial_shot::AerialShot;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn easy_in_front_of_goal() {
        let test = TestRunner::start(
            AerialShot::new(),
            TestScenario {
                ball_loc: Vector3::new(2947.987, 2573.8042, 954.9597),
                ball_vel: Vector3::new(-1540.0411, 924.74066, -1316.2262),
                car_loc: Vector3::new(-1604.453, 2690.225, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009683254, 1.2704238, -0.0000958738),
                car_vel: Vector3::new(357.6586, 1213.9453, 8.309999),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);

        assert!(test.has_scored());
    }
}
