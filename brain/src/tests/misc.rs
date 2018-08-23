use behaviors::RootBehavior;
use collect::ExtendRotation3;
use nalgebra::{Rotation3, Vector3};
use tests::helpers::{TestRunner, TestScenario};

#[test]
fn clear_ball_out_of_goal() {
    let test = TestRunner::start(
        RootBehavior::new(),
        TestScenario {
            ball_loc: Vector3::new(-500.0, -5000.0, 0.0),
            car_loc: Vector3::new(500.0, -5000.0, 17.0),
            car_rot: Rotation3::from_unreal_angles(0.0, 210_f32.to_radians(), 0.0),
            ..Default::default()
        },
    );

    test.sleep_millis(2000);

    let packet = test.sniff_packet();
    assert!(packet.GameBall.Physics.Location.X < -1000.0);
}
