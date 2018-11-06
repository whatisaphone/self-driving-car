use nalgebra::{
    Point2, Point3, Quaternion, Real, Rotation3, Unit, UnitComplex, UnitQuaternion, Vector2,
    Vector3,
};
use physics;
use rlbot;

pub trait ExtendVector2 {
    /// Creates a unit vector in the direction of the given `angle`.
    fn unit(angle: f32) -> Self;
    /// Shorthand for `Unit::new_normalize`.
    fn to_axis(&self) -> Unit<Self>
    where
        Self: Sized;
    fn to_3d(&self, z: f32) -> Vector3<f32>;
    fn uc_angle_to(&self, other: Self) -> UnitComplex<f32>;
    fn angle_to(&self, other: Self) -> f32;
    fn rotation_to(&self, other: Self) -> UnitComplex<f32>;
}

impl ExtendVector2 for Vector2<f32> {
    fn unit(angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Vector2::new(cos, sin)
    }

    fn to_axis(&self) -> Unit<Self> {
        Unit::new_normalize(*self)
    }

    fn to_3d(&self, z: f32) -> Vector3<f32> {
        Vector3::new(self.x, self.y, z)
    }

    // This treats `Vector`s as `Point`s. It should be deprecated.
    fn uc_angle_to(&self, other: Self) -> UnitComplex<f32> {
        let diff = other - self;
        UnitComplex::new(f32::atan2(diff.y, diff.x))
    }

    // This treats `Vector`s as `Point`s. It should be deprecated.
    fn angle_to(&self, other: Self) -> f32 {
        let diff = other - self;
        f32::atan2(diff.y, diff.x)
    }

    fn rotation_to(&self, other: Self) -> UnitComplex<f32> {
        UnitComplex::rotation_between(self, &other)
    }
}

pub trait ExtendVector3 {
    fn to_2d(&self) -> Vector2<f32>;
}

impl ExtendVector3 for Vector3<f32> {
    fn to_2d(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }
}

pub trait ExtendPoint3<N: Real> {
    fn to_2d(&self) -> Point2<N>;
}

impl<N: Real> ExtendPoint3<N> for Point3<N> {
    fn to_2d(&self) -> Point2<N> {
        Point2::new(self.x, self.y)
    }
}

pub trait ExtendUnitComplex {
    fn unit(&self) -> Vector2<f32>;
    /// Convert this complex number (representing a 2D rotation) into a unit
    /// quaternion representing a 3D rotation around the z-axis.
    fn around_z_axis(&self) -> UnitQuaternion<f32>;
}

impl ExtendUnitComplex for UnitComplex<f32> {
    fn unit(&self) -> Vector2<f32> {
        Vector2::new(self.cos_angle(), self.sin_angle())
    }

    fn around_z_axis(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.angle())
    }
}

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
    fn loc_2d(&self) -> Point2<f32>;
    fn rot(&self) -> Rotation3<f32>;
    fn quat(&self) -> UnitQuaternion<f32>;
    fn vel(&self) -> Vector3<f32>;
    fn ang_vel(&self) -> Vector3<f32>;
    /// A unit vector in the car's forward direction.
    fn forward_axis(&self) -> Unit<Vector3<f32>>;
    /// A unit vector in the car's forward direction in 2D space.
    fn forward_axis_2d(&self) -> Unit<Vector2<f32>>;
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

    fn loc_2d(&self) -> Point2<f32> {
        Point2::new(self.Location.X, self.Location.Y)
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

    fn forward_axis_2d(&self) -> Unit<Vector2<f32>> {
        physics::car_forward_axis_2d(self.quat().to_2d())
    }

    fn right_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_right_axis(self.quat())
    }
}

pub trait ExtendUnitQuaternion<N: Real> {
    fn xyzw(x: N, y: N, z: N, w: N) -> Self;

    fn to_2d(&self) -> UnitComplex<N>;

    /// For some reason the games spits out quaternions with some components
    /// negated? I have no idea the underlying cause, or if this is the right
    /// place to fix it, but here we are. This function must be called to
    /// munge/unmunge any quaternions sent to or received from the game.
    ///
    /// There are tests for this near `physicsify`, `convert_quat_to_pyr`, etc.
    fn rocket_league_munge(&self) -> Self;
}

impl<N: Real> ExtendUnitQuaternion<N> for UnitQuaternion<N> {
    fn xyzw(x: N, y: N, z: N, w: N) -> Self {
        UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
    }

    fn to_2d(&self) -> UnitComplex<N> {
        UnitComplex::new(self.scaled_axis().z)
    }

    fn rocket_league_munge(&self) -> Self {
        let coords = self.as_ref().coords;
        // I have no clue what the reasoning behind this is.
        Self::xyzw(-coords.x, -coords.y, coords.z, coords.w)
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
    fn to_3d(&self) -> Unit<Vector3<N>>;
}

impl<N: Real> ExtendUnitVector2<N> for Unit<Vector2<N>> {
    fn rotation_to(&self, other: &Self) -> UnitComplex<N> {
        UnitComplex::rotation_between_axis(self, other)
    }

    fn to_3d(&self) -> Unit<Vector3<N>> {
        Unit::new_unchecked(Vector3::new(self.x, self.y, nalgebra::zero()))
    }
}
