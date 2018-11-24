use nalgebra::UnitQuaternion;
use prelude::*;

pub fn convert_quat_to_pyr(quat: &UnitQuaternion<f32>) -> (f32, f32, f32) {
    quat.to_rotation_matrix().to_unreal_angles()
}

/// I am really not confident about these angle conversions, so let's test as
/// much as possible.
#[cfg(test)]
mod tests {
    use chip;
    use nalgebra::{Rotation3, UnitQuaternion, Vector3};
    use prelude::*;
    use rotation;

    const EPS: f32 = 0.05;

    lazy_static! {
        static ref CASES: Vec<(UnitQuaternion<f32>, (f32, f32, f32))> = vec![
            (
                UnitQuaternion::xyzw(-0.0015563988, 0.0045613004, 0.32138819, 0.94693524),
                (-0.009683254, 0.65443456, 0.0),
            ),
            (
                UnitQuaternion::xyzw(0.37422496, 0.6927582, -0.34245488, 0.51260734),
                (-1.3085815, 2.421868, 2.7907903),
            ),
        ];
    }

    pub fn convert_pyr_to_quat((p, y, r): (f32, f32, f32)) -> UnitQuaternion<f32> {
        let mat = chip::euler_rotation(&Vector3::new(p, y, r));
        let coords = UnitQuaternion::from_rotation_matrix(&mat).unwrap().coords;
        UnitQuaternion::xyzw(-coords.x, -coords.y, -coords.z, coords.w)
    }

    #[test]
    fn convert_quat_to_pyr() {
        for (case_quat, (case_pitch, case_yaw, case_roll)) in &*CASES {
            println!("{:?}", case_quat);
            println!("{:?} {:?} {:?}", case_pitch, case_yaw, case_roll);
            let (pitch, yaw, roll) = rotation::convert_quat_to_pyr(&case_quat);
            assert!((pitch - case_pitch).abs() < EPS, "{}", pitch);
            assert!((yaw - case_yaw).abs() < EPS, "{}", yaw);
            assert!((roll - case_roll).abs() < EPS, "{}", roll);
        }
    }

    #[test]
    fn test_convert_pyr_to_quat() {
        for &(case_quat, (case_pitch, case_yaw, case_roll)) in &*CASES {
            println!("{:?}", case_quat);
            println!("{:?} {:?} {:?}", case_pitch, case_yaw, case_roll);
            let quat = convert_pyr_to_quat((case_pitch, case_yaw, case_roll));
            println!("{:?}", quat);
            assert!(case_quat.rotation_to(&quat).angle() < EPS);
        }
    }

    #[test]
    fn quat_to_unreal_angles() {
        for &(case_quat, (case_pitch, case_yaw, case_roll)) in &*CASES {
            println!("{:?}", case_quat);
            println!("{:?} {:?} {:?}", case_pitch, case_yaw, case_roll);
            let (pitch, yaw, roll) = case_quat.to_rotation_matrix().to_unreal_angles();
            assert!((pitch - case_pitch).abs() < EPS, "{}", pitch);
            assert!((yaw - case_yaw).abs() < EPS, "{}", yaw);
            assert!((roll - case_roll).abs() < EPS, "{}", roll);
        }
    }

    #[test]
    fn from_unreal_angles_to_unreal_angles() {
        for &(_case_quat, (case_pitch, case_yaw, case_roll)) in &*CASES {
            println!("{:?} {:?} {:?}", case_pitch, case_yaw, case_roll);
            let mat = Rotation3::from_unreal_angles(case_pitch, case_yaw, case_roll);
            let (pitch, yaw, roll) = mat.to_unreal_angles();
            assert!((pitch - case_pitch).abs() < EPS, "{}", pitch);
            assert!((yaw - case_yaw).abs() < EPS, "{}", yaw);
            assert!((roll - case_roll).abs() < EPS, "{}", roll);
        }
    }

    #[test]
    #[ignore(note = "I think chip's function is incorrect?")]
    fn chip_from_unreal_angles() {
        for &(_case_quat, (case_pitch, case_yaw, case_roll)) in &*CASES {
            println!("{:?} {:?} {:?}", case_pitch, case_yaw, case_roll);
            let mat1 = chip::euler_rotation(&Vector3::new(case_pitch, case_yaw, case_roll));
            let mat2 = Rotation3::from_unreal_angles(case_pitch, case_yaw, case_roll);
            eprintln!("mat1 = {:?}", mat1);
            eprintln!("mat2 = {:?}", mat2);
            assert!(mat1.rotation_to(&mat2).angle() < EPS);
        }
    }

    #[test]
    #[ignore(note = "I think chip's function is incorrect?")]
    fn chip_to_unreal_angles() {
        for &(_case_quat, (case_pitch, case_yaw, case_roll)) in &*CASES {
            println!("{:?} {:?} {:?}", case_pitch, case_yaw, case_roll);
            let mat1 = chip::euler_rotation(&Vector3::new(case_pitch, case_yaw, case_roll));
            let (pitch, yaw, roll) = mat1.to_unreal_angles();
            assert!((pitch - case_pitch).abs() < EPS, "{}", pitch);
            assert!((yaw - case_yaw).abs() < EPS, "{}", yaw);
            assert!((roll - case_roll).abs() < EPS, "{}", roll);
        }
    }

    // See also the tests for `dom::to_rotation_matrix`.
}
