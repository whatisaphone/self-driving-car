#[cfg(test)]
mod integration_tests {
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::Runner2;

    #[test]
    #[ignore] // TODO
    fn basic() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-2655.6062, 1005.91815, 102.159805),
            ball_vel: Vector3::new(-73.839806, -1789.6954, 0.41171908),
            car_loc: Vector3::new(658.7359, -3706.0732, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 2.8636546, 0.0),
            car_vel: Vector3::new(-1999.5629, 569.28217, 8.3),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }
}
