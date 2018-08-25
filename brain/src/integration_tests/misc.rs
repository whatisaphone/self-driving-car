use behavior::RootBehavior;
use collect::ExtendRotation3;
use integration_tests::helpers::{TestRunner, TestScenario};
use nalgebra::{Rotation3, Vector3};

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

#[test]
#[ignore]
// We came in too hot, we should be able to smack it to the side.
fn todo() {
    let test = TestRunner::start(
        RootBehavior::new(),
        TestScenario::from_collect_row("162.00925	-1004.2267	-1863.0571	93.15	1.0648702	-2.6295307	0.66037875	1196.1945	-1186.7386	0	4.445553	3.9708207	-0.68442774	1692.9968	-2508.7695	17.01	-0.009779127	-2.0910075	0	-896.0074	-1726.876	8.375226	-0.002352258	0.0009174193	-1.4248774	-1994.5284	-884.29803	17.01	-0.009491506	-0.56105345	0	1013.4857	-641.1561	8.201742	0.0002	0.00028258064	0.19836259"),
    );

    test.sleep_millis(3000);
}
