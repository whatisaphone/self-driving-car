use behavior::shoot::Shoot;
use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::simple_steer_towards;
use predict::estimate_intercept_car_ball_2;
use rlbot;
use simulate::rl;
use utils::{one_v_one, ExtendPhysics, ExtendVector3};

pub struct Offense {
    min_distance: Option<f32>,
}

impl Offense {
    pub fn new() -> Offense {
        Offense { min_distance: None }
    }
}

impl Behavior for Offense {
    fn name(&self) -> &'static str {
        "Offense"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if !GetToFlatGround::on_flat_ground(packet) {
            return Action::call(GetToFlatGround::new());
        }

        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |t, &loc, vel| {
            Shoot::good_angle(loc)
        });

        let good_angle = Shoot::good_angle(intercept.ball_loc);
        if !good_angle {
            // Stop near the perimeter and wait
            //
            // This is VERY temporary code. It should be replaced with other "idle" tasks
            // like getting boost or lining up with the predicted ball in a place where we
            // can do something useful, etc
            eeg.draw(Drawable::print("Waiting for a good angle", color::GREEN));
            return Action::Yield(rlbot::PlayerInput {
                Throttle: if me.Physics.loc().y.abs() >= rl::FIELD_MAX_Y - 2000.0 {
                    if me.Physics.vel().y >= 250.0 {
                        -1.0
                    } else {
                        0.0
                    }
                } else {
                    1.0
                },
                Steer: simple_steer_towards(&me.Physics, intercept.ball_loc.to_2d()),
                ..Default::default()
            });
        }

        eeg.log(format!("Good angle found {:?}", intercept.ball_loc));
        return Action::call(Shoot::new());
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::offense::Offense;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn wait_for_curl_around_lip_near_post() {
        let test = TestRunner::start(
            Offense::new(),
            TestScenario {
                ball_loc: Vector3::new(2972.6848, 1018.38824, 101.33544),
                ball_vel: Vector3::new(-1029.0707, 2168.4673, -61.355755),
                car_loc: Vector3::new(3147.7668, 686.3356, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.689584, 0.0),
                car_vel: Vector3::new(-685.4966, 1531.4093, 83.56691),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }
}
