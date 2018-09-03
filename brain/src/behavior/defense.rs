use behavior::{Action, Behavior};
use eeg::{Drawable, EEG};
use maneuvers::BounceShot;
use predict::estimate_intercept_car_ball;
use rlbot;
use utils::{
    enemy_goal_center, my_car, my_goal_center_2d, own_goal_left_post, own_goal_right_post,
    ExtendPhysics, ExtendVector2, ExtendVector3,
};

pub struct Defense {
    finished: bool,
}

impl Defense {
    pub fn new() -> Defense {
        Defense { finished: false }
    }
}

impl Behavior for Defense {
    fn name(&self) -> &'static str {
        "Defense"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if self.finished {
            return Action::Return;
        }

        let me = my_car(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        eeg.log("redirect to own corner");
        let me_loc = me.Physics.loc().to_2d();
        let angle_to_own_goal = me_loc.angle_to(enemy_goal_center());
        let angle_to_ball_intercept = me_loc.angle_to(intercept.ball_loc.to_2d());
        let target_loc = if angle_to_own_goal > angle_to_ball_intercept {
            eeg.log("push from left to right");
            my_goal_center_2d() + (own_goal_right_post() - my_goal_center_2d()) * 4.0
        } else {
            eeg.log("push from right to left");
            my_goal_center_2d() + (own_goal_left_post() - my_goal_center_2d()) * 4.0
        };

        eeg.draw(Drawable::GhostBall(intercept.ball_loc));

        self.finished = true;
        Action::call(BounceShot::new().with_target_loc(target_loc))
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::RootBehavior;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    fn bouncing_save() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-3143.9788, -241.96017, 1023.1816),
            ball_vel: Vector3::new(717.56323, -1200.3536, 331.91443),
            car_loc: Vector3::new(-4009.9998, -465.8022, 86.914),
            car_rot: Rotation3::from_unreal_angles(-0.629795, -0.7865487, 0.5246214),
            car_vel: Vector3::new(982.8443, -1059.1908, -935.80194),
            ..Default::default()
        });

        let start_time = test.sniff_packet().GameInfo.TimeSeconds;
        test.set_behavior(RootBehavior::new());

        let mut max_z = 0.0_f32;
        loop {
            let packet = test.sniff_packet();
            let elapsed = packet.GameInfo.TimeSeconds - start_time;
            if elapsed >= 4.0 {
                break;
            }
            if elapsed >= 1.0 && packet.GameBall.Physics.Velocity.Z > 0.0 {
                max_z = max_z.max(packet.GameBall.Physics.Location.Z);
            }
        }

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "Defense"));
            assert!(eeg.log.iter().any(|x| x == "redirect to own corner"));
            assert!(eeg.log.iter().any(|x| x == "push from left to right"));
        });

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameBall.Physics.Location);
        assert!(packet.GameBall.Physics.Location.X >= 1000.0, "{:?}");

        // Should power-shot, meaning the ball bounces high.
        assert!(max_z >= 400.0, "{}", max_z);
    }
}
