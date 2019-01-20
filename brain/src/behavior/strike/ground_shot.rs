#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::helpers::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    fn crossing_the_midfield() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1794.4557, -681.9332, 99.93823),
                ball_vel: Vector3::new(-619.51764, 1485.6294, -12.806913),
                car_loc: Point3::new(-3472.8125, -1983.225, 16.937647),
                car_rot: Rotation3::from_unreal_angles(-0.009779127, 2.4388378, -0.0011504856),
                car_vel: Vector3::new(-1599.1952, 1223.4504, 9.51471),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(4000);

        assert!(test.has_scored());
    }

    #[test]
    fn crossing_the_box() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-726.1142, -673.77716, 118.28892),
                ball_vel: Vector3::new(1032.4805, 1531.884, -72.43818),
                car_loc: Point3::new(-45.566628, -1993.5394, 16.711021),
                car_rot: Rotation3::from_unreal_angles(-0.010258497, 0.60458016, 0.0013422332),
                car_vel: Vector3::new(1566.5747, 1017.1486, 13.497895),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3500);
        assert!(test.has_scored());
    }

    #[test]
    fn high_bouncer() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1725.8822, 4719.4307, 93.15),
                ball_vel: Vector3::new(1031.4242, 2151.6794, 0.0),
                car_loc: Point3::new(-2374.2222, 3805.5469, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009970875, 1.0610354, -0.0002876214),
                car_vel: Vector3::new(521.8343, 928.79755, 8.326952),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(4000);
        assert!(test.has_scored());
    }

    #[test]
    fn easy_open_net() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(999.651, 3636.9731, 93.14),
                ball_vel: Vector3::new(-271.7422, -1642.4099, 0.0),
                car_loc: Point3::new(1981.3068, -3343.5154, 16.99),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.9184347, 0.0),
                car_vel: Vector3::new(-544.83453, 1537.2355, 8.53),
                boost: 0,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(5000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore(note = "when the angle is too tight")]
    fn tight_angle_needs_correction() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2618.1267, 4567.453, 93.14),
                ball_vel: Vector3::new(204.82155, -438.9531, 0.0),
                car_loc: Point3::new(-3850.746, 3749.8147, 16.319502),
                car_rot: Rotation3::from_unreal_angles(-0.15867114, -0.33191508, 0.005273059),
                car_vel: Vector3::new(1287.4675, -433.82834, -183.28568),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(2000);
        assert!(test.has_scored());
    }

    #[test]
    fn corner_shot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2616.377, 4173.1816, 122.709236),
                ball_vel: Vector3::new(662.0207, -114.385414, 294.32352),
                car_loc: Point3::new(-3791.579, 2773.0996, 14.34),
                car_rot: Rotation3::from_unreal_angles(0.013038836, 0.08504006, -0.0035473306),
                car_vel: Vector3::new(1109.654, 62.572224, 22.532219),
                boost: 0,
                ..Default::default()
            })
            .soccar()
            .run_for_millis(3000);

        assert!(test.has_scored());
    }

    #[test]
    fn rolling_to_corner_of_goal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(151.63426, 4371.35, 93.15),
                ball_vel: Vector3::new(-435.87555, 272.40097, 0.0),
                car_loc: Point3::new(-2071.033, 4050.8577, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009491506, -0.61694795, 0.0),
                car_vel: Vector3::new(232.67545, -405.04382, 8.36),
                ..Default::default()
            })
            .soccar()
            .run_for_millis(2000);

        assert!(test.has_scored());
    }

    #[test]
    fn tight_angle_with_no_boost() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2815.55, 4677.98, 233.65),
                ball_rot: Rotation3::from_unreal_angles(-0.3859388, -0.9047023, 2.4510245),
                ball_vel: Vector3::new(882.251, -325.971, -76.541),
                car_loc: Point3::new(-3025.66, 3546.3599, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009533787, 0.71604276, -0.000059155664),
                car_vel: Vector3::new(820.001, 709.05096, 8.311),
                ..Default::default()
            })
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(2500);

        assert!(test.has_scored());
    }

    #[test]
    fn tight_angle_with_boost() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2815.55, 4677.98, 233.65),
                ball_rot: Rotation3::from_unreal_angles(-0.3859388, -0.9047023, 2.4510245),
                ball_vel: Vector3::new(882.251, -325.971, -76.541),
                car_loc: Point3::new(-3025.66, 3546.3599, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009533787, 0.71604276, -0.000059155664),
                car_vel: Vector3::new(820.001, 709.05096, 8.311),
                ..Default::default()
            })
            .starting_boost(50.0)
            .soccar()
            .run_for_millis(2500);

        assert!(test.has_scored());
    }
}
