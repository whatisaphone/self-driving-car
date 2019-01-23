#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    #[ignore(note = "TODO")]
    fn basic() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2655.6062, 1005.91815, 102.159805),
                ball_vel: Vector3::new(-73.839806, -1789.6954, 0.41171908),
                car_loc: Point3::new(658.7359, -3706.0732, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 2.8636546, 0.0),
                car_vel: Vector3::new(-1999.5629, 569.28217, 8.3),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(5000);

        assert!(test.has_scored());
    }
}
