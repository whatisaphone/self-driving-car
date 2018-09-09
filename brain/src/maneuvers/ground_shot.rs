use behavior::{Action, Behavior};
use collect::ExtendRotation3;
use eeg::{color, Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::{simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use nalgebra::Vector3;
use predict::estimate_intercept_car_ball_2;
use rlbot;
use simulate::rl;
use std::f32::consts::PI;
use utils::{
    enemy_goal_center, enemy_goal_left_post, enemy_goal_right_post, one_v_one, ExtendPhysics,
    ExtendVector2, ExtendVector3,
};

pub struct GroundShot {
    min_distance: Option<f32>,
    finished: bool,
}

impl GroundShot {
    pub fn new() -> GroundShot {
        GroundShot {
            min_distance: None,
            finished: false,
        }
    }

    pub fn good_angle(ball_loc: Vector3<f32>) -> bool {
        // let angle_l = ball_loc.to_2d().angle_to(enemy_goal_left_post());
        // let angle_r = ball_loc.to_2d().angle_to(enemy_goal_right_post());

        // This is woefully incomplete
        if ball_loc.x.abs() >= rl::FIELD_MAX_X || ball_loc.y.abs() >= rl::FIELD_MAX_Y {
            false // Ball is outside the field; clearly prediction has gone wrong somehow.
        } else if ball_loc.y.abs() >= rl::FIELD_MAX_Y - 250.0 && ball_loc.x.abs() >= rl::GOALPOST_X
        {
            false
        } else {
            true
        }
    }
}

impl Behavior for GroundShot {
    fn name(&self) -> &'static str {
        "GroundShot"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if self.finished {
            return Action::Return;
        }

        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |t, &loc, vel| {
            loc.z < 110.0 && Self::good_angle(loc)
        });

        if !Self::good_angle(intercept.ball_loc) {
            eeg.log(format!("Bad angle from {:?}", intercept.ball_loc));
            return Action::Return;
        }

        let target_loc = intercept.ball_loc.to_2d()
            + (intercept.ball_loc.to_2d() - enemy_goal_center()).normalize() * 150.0;
        let target_dist = (target_loc - me.Physics.loc().to_2d()).norm();

        // If the ball has moved further away, assume we hit it and we're done.
        match self.min_distance {
            Some(md) if target_dist >= md * 2.0 => return Action::Return,
            _ => self.min_distance = Some(target_dist),
        }

        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostBall(intercept.ball_loc));

        // This behavior currently just operates in 2D
        if !GetToFlatGround::on_flat_ground(packet) {
            return Action::call(GetToFlatGround::new());
        }

        if target_dist <= 250.0 {
            self.finished = true;
            return shoot(packet, eeg);
        }

        // TODO: this is not how this works…
        let mut child =
            GroundAccelToLoc::new(target_loc, packet.GameInfo.TimeSeconds + intercept.time);
        child.execute(packet, eeg)
    }
}

fn shoot(packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
    let (me, _enemy) = one_v_one(packet);
    let angle = simple_yaw_diff(&me.Physics, enemy_goal_center());
    if angle.abs() >= PI / 2.0 {
        eeg.log("Incorrect approach angle");
        return Action::Return;
    }

    return Action::call(QuickJumpAndDodge::begin(packet).yaw(angle));
}

#[cfg(test)]
mod integration_tests {
    use behavior::runner::PUSHED;
    use behavior::RootBehavior;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::bounce_shot::BounceShot;
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn crossing_the_midfield() {
        let test = TestRunner::start(
            RootBehavior::new(),
            TestScenario {
                ball_loc: Vector3::new(-1794.4557, -681.9332, 99.93823),
                ball_vel: Vector3::new(-619.51764, 1485.6294, -12.806913),
                car_loc: Vector3::new(-3472.8125, -1983.225, 16.937647),
                car_rot: Rotation3::from_unreal_angles(-0.009779127, 2.4388378, -0.0011504856),
                car_vel: Vector3::new(-1599.1952, 1223.4504, 9.51471),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);
        test.examine_eeg(|eeg| {
            assert!(
                eeg.log
                    .iter()
                    .any(|x| *x == format!("{} GroundShot", PUSHED))
            );
        });
        assert!(test.has_scored());
    }
}
