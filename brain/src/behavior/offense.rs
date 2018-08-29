use behavior::shoot::Shoot;
use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

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
        eeg.draw(Drawable::print(self.name(), color::YELLOW));

        // TODO: this is not how this works…
        let mut child = Shoot::new();
        child.execute(packet, eeg)
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::offense::Offense;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
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
