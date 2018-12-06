#[cfg(test)]
mod integration_tests {
    use crate::{
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::Runner2,
    };
    use common::prelude::*;
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn choose_a_side() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(1042.4299, 1818.3915, 93.14645),
            ball_vel: Vector3::new(0.34508505, -765.4985, -6.216589),
            car_loc: Vector3::new(920.5995, 3787.2036, 44.447636),
            car_rot: Rotation3::from_unreal_angles(-0.010929613, -1.1549916, -1.3353302),
            car_vel: Vector3::new(63.52187, -1216.5942, 40.226746),
            ..Default::default()
        });
        test.set_behavior(Runner2::soccar());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn fast_push_to_corner() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-3231.1572, 1764.513, 92.159996),
            ball_vel: Vector3::new(800.4098, -1559.2743, 0.0),
            car_loc: Vector3::new(-3508.7988, 3034.4133, 17.02),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 3.0207918, 0.0),
            car_vel: Vector3::new(-1254.4124, 211.01375, 8.24),
            ..Default::default()
        });
        test.set_behavior(Runner2::soccar());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }
}
