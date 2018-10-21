use nalgebra::{Point3, Real, Rotation3, Unit, UnitComplex, UnitQuaternion, Vector2, Vector3};
use physics;
use rlbot;

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

pub trait ExtendPhysics {
    fn loc(&self) -> Vector3<f32>;
    fn locp(&self) -> Point3<f32>;
    fn rot(&self) -> Rotation3<f32>;
    fn quat(&self) -> UnitQuaternion<f32>;
    fn vel(&self) -> Vector3<f32>;
    fn ang_vel(&self) -> Vector3<f32>;
    /// A unit vector in the car's forward direction.
    fn forward_axis(&self) -> Unit<Vector3<f32>>;
    /// A unit vector in the car's right direction.
    fn right_axis(&self) -> Unit<Vector3<f32>>;
}

impl ExtendPhysics for rlbot::ffi::Physics {
    fn loc(&self) -> Vector3<f32> {
        Vector3::new(self.Location.X, self.Location.Y, self.Location.Z)
    }

    fn locp(&self) -> Point3<f32> {
        Point3::new(self.Location.X, self.Location.Y, self.Location.Z)
    }

    fn rot(&self) -> Rotation3<f32> {
        Rotation3::from_unreal_angles(self.Rotation.Pitch, self.Rotation.Yaw, self.Rotation.Roll)
    }

    fn quat(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_rotation_matrix(&self.rot())
    }

    fn vel(&self) -> Vector3<f32> {
        Vector3::new(self.Velocity.X, self.Velocity.Y, self.Velocity.Z)
    }

    fn ang_vel(&self) -> Vector3<f32> {
        Vector3::new(
            self.AngularVelocity.X,
            self.AngularVelocity.Y,
            self.AngularVelocity.Z,
        )
    }

    fn forward_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_forward_axis(self.quat())
    }

    fn right_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_right_axis(self.quat())
    }
}

pub trait ExtendUnitVector3<N: Real> {
    fn to_2d(&self) -> Unit<Vector2<N>>;
    fn rotation_to(&self, other: &Self) -> UnitQuaternion<N>;
}

impl<N: Real> ExtendUnitVector3<N> for Unit<Vector3<N>> {
    fn to_2d(&self) -> Unit<Vector2<N>> {
        Unit::new_normalize(Vector2::new(self.x, self.y))
    }

    fn rotation_to(&self, other: &Self) -> UnitQuaternion<N> {
        UnitQuaternion::rotation_between_axis(self, other).unwrap()
    }
}

pub trait ExtendUnitVector2<N: Real> {
    fn rotation_to(&self, other: &Self) -> UnitComplex<N>;
}

impl<N: Real> ExtendUnitVector2<N> for Unit<Vector2<N>> {
    fn rotation_to(&self, other: &Self) -> UnitComplex<N> {
        UnitComplex::rotation_between_axis(self, other)
    }
}
