#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::helpers::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    #[ignore(note = "TODO")]
    fn easy_in_front_of_goal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2947.987, 2573.8042, 954.9597),
                ball_vel: Vector3::new(-1540.0411, 924.74066, -1316.2262),
                car_loc: Point3::new(-1604.453, 2690.225, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009683254, 1.2704238, -0.0000958738),
                car_vel: Vector3::new(357.6586, 1213.9453, 8.309999),
                ..Default::default()
            })
            .run();

        test.sleep_millis(4000);

        assert!(test.has_scored());
    }
}
