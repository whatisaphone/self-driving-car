use behavior::{Action, Behavior};
use eeg::EEG;
use rlbot;

pub struct JumpShot;

impl JumpShot {
    pub fn new() -> JumpShot {
        JumpShot
    }
}

impl Behavior for JumpShot {
    fn name(&self) -> &'static str {
        "JumpShot"
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        Action::Yield(Default::default())
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::jump_shot::JumpShot;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
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
}
