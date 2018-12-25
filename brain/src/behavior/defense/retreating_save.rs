#[cfg(test)]
mod integration_tests {
    use crate::{
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::Runner,
    };
    use common::{prelude::*, rl};
    use nalgebra::{Point2, Point3, Rotation3, Vector3};

    #[test]
    fn falling_in_front_of_far_corner() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(882.9138, -5002.2944, 608.2664),
                ball_vel: Vector3::new(-211.04604, 37.17434, 459.58438),
                car_loc: Point3::new(-2512.3357, -2450.706, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009683254, -0.68204623, -0.0000958738),
                car_vel: Vector3::new(786.13666, -620.0981, 8.309999),
                ..Default::default()
            })
            .behavior(Runner::soccar())
            .run_for_millis(2500);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        let own_goal = Point2::new(0.0, -rl::FIELD_MAX_Y);
        let goal_to_ball_dist = (packet.GameBall.Physics.loc_2d() - own_goal).norm();
        assert!(goal_to_ball_dist >= 750.0);
        assert!(packet.GameBall.Physics.vel().norm() >= 1000.0);
    }

    #[test]
    #[ignore] // TODO
    fn rolling_quickly() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(2792.5564, 2459.176, 94.02834),
                ball_vel: Vector3::new(-467.7808, -2086.822, -88.445175),
                car_loc: Point3::new(3001.808, 3554.98, 16.99),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.7710767, 0.0000958738),
                car_vel: Vector3::new(-379.28546, -1859.9683, 8.41),
                ..Default::default()
            })
            .behavior(Runner::soccar())
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn rolling_around_corner_into_box() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Point3::new(3042.6016, -4141.044, 180.57321),
            ball_vel: Vector3::new(-1414.86847, -1357.0486, -0.0),
            car_loc: Point3::new(720.54016, 635.665, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.4134674, 0.0),
            car_vel: Vector3::new(256.23804, -1591.1218, 8.3),
            ..Default::default()
        });
        test.set_behavior(Runner::soccar());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn low_bouncing_directly_ahead() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Point3::new(-916.57043, -5028.2397, 449.42386),
            ball_vel: Vector3::new(215.22325, 0.07279097, -403.102),
            car_loc: Point3::new(-320.59094, -2705.4436, 17.02),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -1.6579456, 0.0),
            car_vel: Vector3::new(-85.847946, -990.35706, 8.0),
            ..Default::default()
        });
        test.set_behavior(Runner::soccar());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn high_loft_in_front_of_goal() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Point3::new(-2285.6035, -5024.131, 438.6606),
            ball_vel: Vector3::new(751.0301, 16.736507, 811.52356),
            car_loc: Point3::new(-1805.5178, -2341.8872, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -0.4485935, 0.0),
            car_vel: Vector3::new(1141.101, -487.77042, 8.34),
            ..Default::default()
        });
        test.set_behavior(Runner::soccar());
        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn loft_in_front_of_goal_from_the_side() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2288.2634, -4688.248, 93.15),
                ball_vel: Vector3::new(1281.6293, -1659.181, 0.0),
                car_loc: Point3::new(-3077.711, -3389.5276, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -0.95528656, -0.0000958738),
                car_vel: Vector3::new(1027.5283, -1455.2512, 8.3),
                ..Default::default()
            })
            .behavior(Runner::soccar())
            .run_for_millis(4000);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.loc().x >= 1000.0);
        assert!(packet.GameBall.Physics.vel().x >= 1000.0);
    }
}