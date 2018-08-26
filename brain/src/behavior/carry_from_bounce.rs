#[cfg(test)]
mod integration_tests {
    use behavior::RootBehavior;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn wait_for_side_wall_rebound() {
        let test = TestRunner::start(
            RootBehavior::new(),
            TestScenario {
                ball_loc: Vector3::new(-2591.3115, -1322.5367, 459.51404),
                ball_vel: Vector3::new(-930.0902, -258.3213, -359.45007),
                car_loc: Vector3::new(-3092.0574, -3168.787, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -3.0723717, 0.0),
                car_vel: Vector3::new(-1631.8348, -197.60046, 8.34),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }
}
