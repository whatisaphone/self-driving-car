use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::{AerialLocTime, GetToFlatGround};
use mechanics::{simple_yaw_diff, GroundAccelToLoc};
use predict::estimate_intercept_car_ball_2;
use rlbot;
use simulate::{rl, CarAerial60Deg};
use utils::{enemy_goal_center, my_car, one_v_one, ExtendPhysics, ExtendVector3};

pub struct AerialShot {
    finished: bool,
}

impl AerialShot {
    pub fn new() -> AerialShot {
        AerialShot { finished: false }
    }
}

impl Behavior for AerialShot {
    fn name(&self) -> &'static str {
        stringify!(AerialShot)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if self.finished {
            return Action::Return;
        }

        let (me, _enemy) = one_v_one(packet);
        let ball_dist = (me.Physics.loc() - packet.GameBall.Physics.loc()).norm();

        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |t, loc, vel| {
            let max_comfortable_z = rl::CROSSBAR_Z + enemy_goal_center().y - loc.y;
            if loc.z >= max_comfortable_z {
                return false;
            }
            let cost = CarAerial60Deg::cost(loc.z);
            cost.time < t
        });

        let cost = CarAerial60Deg::cost(intercept.ball_loc.z);

        eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        eeg.log(format!("intercept_ball_z: {:.0}", intercept.ball_loc.z));
        eeg.log(format!("intercept_time: {:.2}", intercept.time));
        eeg.log(format!("aerial_time: {:.2}", cost.time));

        self.finished = true;
        Action::call(AerialLocTime::new(
            intercept.ball_loc,
            packet.GameInfo.TimeSeconds + intercept.time,
        ))
    }
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
