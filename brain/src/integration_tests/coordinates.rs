/// Ensure coordinates are represented correctly in various cases.
use bakkesmod::BakkesMod;
use behavior::NullBehavior;
use collect::ExtendRotation3;
use integration_tests::helpers::{TestRunner, TestScenario};
use nalgebra::{Rotation3, Vector3};
use std::f32::consts::PI;
use utils::geometry::ExtendVector3;
use utils::ExtendPhysics;

#[test]
#[ignore] // Very unlikely to break
fn location() {
    let cases = [
        ("0 0 500", 0.0, 0.0, 500.0),
        ("1000 0 500", 1000.0, 0.0, 500.0),
        ("0 1000 500", 0.0, 1000.0, 500.0),
    ];
    for &(bakkesmod_loc, x, y, z) in cases.iter() {
        let expected = Vector3::new(x, y, z);
        let test = TestRunner::start(
            NullBehavior,
            format!(
                "player 0 location {}; player 0 velocity 0 0 0",
                bakkesmod_loc
            ),
        );

        let packet = test.sniff_packet();
        let car = packet.GameCars[0].Physics;
        println!("{:?} {} {}", car.Location, car.loc(), expected);
        assert!((x, y) == (car.Location.X, car.Location.Y));
        assert!((car.loc() - expected).to_2d().norm() < 1.0);
    }
}

#[test]
#[ignore] // Very unlikely to break
fn rotation() {
    let cases = [
        ("0 0 0", 0.0, 0.0, 0.0),
        ("0 16384 0", 0.0, PI / 2.0, 0.0),
        ("0 -32768 0", 0.0, -PI, 0.0),
        ("16384 0 0", PI / 2.0, 0.0, 0.0),
        ("0 0 16384", 0.0, 0.0, PI / 2.0),
    ];
    for &(bakkesmod_rot, pitch, yaw, roll) in cases.iter() {
        let expected = Rotation3::from_unreal_angles(pitch, yaw, roll);
        let test = TestRunner::start(
            NullBehavior,
            format!(
                "player 0 location 0 0 1000; player 0 rotation {}; player 0 velocity 0 0 0",
                bakkesmod_rot
            ),
        );

        let packet = test.sniff_packet();
        let car = packet.GameCars[0].Physics;
        assert!(car.rot().angle_to(&expected) < 1.0_f32.to_radians());
    }
}

#[test]
#[ignore] // Very unlikely to break
fn rotation_round_trip() {
    let rots = [
        Rotation3::from_unreal_angles(0.0, 0.0, 0.0),
        Rotation3::from_unreal_angles(0.0, 1.0, 0.0),
        Rotation3::from_unreal_angles(1.0, 0.0, 0.0),
        Rotation3::from_unreal_angles(-1.0, 0.0, -1.0),
    ];
    for &rot in rots.iter() {
        let test = TestRunner::start(
            NullBehavior,
            TestScenario {
                car_loc: Vector3::new(0.0, 0.0, 1000.0),
                car_rot: rot,
                ..Default::default()
            },
        );

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Physics.rot().angle_to(&rot) < 1.0_f32.to_radians());
    }
}
