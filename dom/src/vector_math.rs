// Source: https://github.com/DomNomNom/RocketBot/blob/c7553453acf4d0c3ff286cc2d9b44f9cc3a784b4/vector_math.py

use nalgebra::{Matrix3, Rotation3};

pub fn to_rotation_matrix(pitch: f32, yaw: f32, roll: f32) -> Rotation3<f32> {
    // Note: Unreal engine coordinate system
    let y = pitch;
    let cosy = y.cos();
    let siny = y.sin();
    #[rustfmt::skip]
    let mat_pitch = Matrix3::new(
        cosy, 0.0, -siny,
        0.0,  1.0, 0.0,
        siny, 0.0, cosy,
    );

    let z = yaw;
    let cosz = z.cos();
    let sinz = z.sin();
    #[rustfmt::skip]
    let mat_yaw = Matrix3::new(
        cosz, -sinz, 0.0,
        sinz, cosz,  0.0,
        0.0,  0.0,   1.0,
    );

    let x = roll;
    let cosx = x.cos();
    let sinx = x.sin();
    #[rustfmt::skip]
    let mat_roll = Matrix3::new(
        1.0, 0.0,   0.0,
        0.0, cosx,  sinx,
        0.0, -sinx, cosx,
    );

    Rotation3::from_matrix_unchecked(mat_yaw * mat_pitch * mat_roll)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::prelude::*;
    use nalgebra::Vector3;

    const EPS: f32 = 0.001;

    #[test]
    #[ignore = "I think chip's function is incorrect?"]
    fn same_as_chip() {
        let chip = chip::euler_rotation(&Vector3::new(0.1, 0.2, 0.3));
        let dom = to_rotation_matrix(0.1, 0.2, 0.3);
        println!("chip = {:?}", chip);
        println!("dom = {:?}", dom);
        assert!(chip.angle_to(&dom) < EPS);
    }

    #[test]
    fn from_unreal_angles() {
        let dom = to_rotation_matrix(0.1, 0.2, 0.3);
        let fua = Rotation3::from_unreal_angles(0.1, 0.2, 0.3);
        println!("dom = {:?}", dom);
        println!("fua = {:?}", fua);
        assert!(fua.angle_to(&dom) < EPS);
    }
}
