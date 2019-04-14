#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    #[ignore = "TODO (this could be either a bounce dribble or normal dribble)"]
    fn full_field() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1977.3411, -3995.6826, 93.15),
                ball_vel: Vector3::new(-341.7461, -422.02698, 0.0),
                car_loc: Point3::new(-2438.1355, -5076.892, 87.48564),
                car_rot: Rotation3::from_unreal_angles(-0.9632441, 2.1318498, -0.38071483),
                car_vel: Vector3::new(-413.28445, 545.5142, -830.557),
                boost: 0,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(5000);

        assert!(test.has_scored());
    }
}
