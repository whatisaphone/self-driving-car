#[cfg(test)]
mod integration_tests {
    use crate::integration_tests::helpers::{TestRunner, TestScenario};
    use common::prelude::*;
    use nalgebra::Point3;

    #[test]
    #[ignore] // This basically works but is inaccurate. See the comment above `air()`.
    fn simple() {
        let expected_loc = Point3::new(400.0, 1100.0, 600.0);
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1000.0, 0.0, 0.0),
                ..Default::default()
            })
            .run();

        test.sleep_millis(3000);
        let packet = test.sniff_packet();
        let car_loc = packet.GameCars[0].Physics.loc();
        let distance = (car_loc - expected_loc).norm();
        println!("expected_loc: {:?}", expected_loc);
        println!("car_loc: {:?}", car_loc);
        println!("distance: {:?}", distance);
        assert!(distance < 50.0);
    }
}
