// This stuff does not belong in this crate. I should bite the bullet and break
// out a `utils` crate.
//
// Note that there are some integration tests for ExtendRotation3 in
// brain::integration_tests::coordinates – as I said, this is all misplaced and
// a shuffle is in order.

use nalgebra::Rotation3;

pub trait ExtendRotation3 {
    fn from_unreal_angles(pitch: f32, yaw: f32, roll: f32) -> Rotation3<f32>;
    fn to_unreal_angles(&self) -> (f32, f32, f32);
    fn pitch(&self) -> f32;
    fn yaw(&self) -> f32;
    fn roll(&self) -> f32;
}

// There are three different rotation conventions we need to deal with:
//
// 1. `Rotation3::from_euler_angles` – (roll, pitch, yaw).
// 2. `Rotation3::to_euler_angles` – (roll, yaw, pitch).
// 3. Unreal itself – (yaw, pitch, roll).
//
// This situation is not tenable, so I reimplemented an the conversions using
// Unreal's convention.
impl ExtendRotation3 for Rotation3<f32> {
    fn from_unreal_angles(pitch: f32, yaw: f32, roll: f32) -> Rotation3<f32> {
        Rotation3::from_euler_angles(roll, pitch, yaw)
    }

    fn to_unreal_angles(&self) -> (f32, f32, f32) {
        let (roll, pitch, yaw) = self.to_euler_angles();
        (pitch, yaw, roll)
    }

    fn pitch(&self) -> f32 {
        let (pitch, _yaw, _roll) = self.to_unreal_angles();
        pitch
    }

    fn yaw(&self) -> f32 {
        let (_pitch, yaw, _roll) = self.to_unreal_angles();
        yaw
    }

    fn roll(&self) -> f32 {
        let (_pitch, _yaw, roll) = self.to_unreal_angles();
        roll
    }
}
