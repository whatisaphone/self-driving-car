use crate::physics;
use nalgebra::{
    Point2, Point3, Quaternion, Real, Rotation3, Unit, UnitComplex, UnitQuaternion, Vector2,
    Vector3,
};
use std::{error::Error, mem};

pub trait ExtendVector2<N: Real> {
    /// Creates a unit vector in the direction of the given `angle`.
    fn unit(angle: N) -> Self;
    /// Rotate this vector 90° to the right.
    fn ortho(&self) -> Self;
    /// Shorthand for `Unit::new_normalize`.
    fn to_axis(&self) -> Unit<Self>
    where
        Self: Sized;
    fn to_3d(&self, z: N) -> Vector3<N>;
    fn rotation_to(&self, other: &Self) -> UnitComplex<N>;
    fn angle_to(&self, other: &Self) -> N;
}

impl<N: Real> ExtendVector2<N> for Vector2<N> {
    fn unit(angle: N) -> Self {
        let (sin, cos) = angle.sin_cos();
        Vector2::new(cos, sin)
    }

    fn ortho(&self) -> Self {
        Vector2::new(self.y, -self.x)
    }

    fn to_axis(&self) -> Unit<Self> {
        Unit::new_normalize(*self)
    }

    fn to_3d(&self, z: N) -> Vector3<N> {
        Vector3::new(self.x, self.y, z)
    }

    fn rotation_to(&self, other: &Self) -> UnitComplex<N> {
        UnitComplex::rotation_between(self, other)
    }

    fn angle_to(&self, other: &Self) -> N {
        self.rotation_to(other).angle()
    }
}

pub trait ExtendVector3 {
    /// Shorthand for `Unit::new_normalize`.
    fn to_axis(&self) -> Unit<Self>
    where
        Self: Sized;
    fn to_2d(&self) -> Vector2<f32>;
}

impl ExtendVector3 for Vector3<f32> {
    fn to_axis(&self) -> Unit<Self> {
        Unit::new_normalize(*self)
    }

    fn to_2d(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }
}

pub trait ExtendPoint2<N: Real> {
    fn to_3d(&self, z: N) -> Point3<N>;
    // This uses an implicit origin vector and should be considered deprecated.
    fn negated_difference_and_angle_to(&self, other: Self) -> N;
}

impl<N: Real> ExtendPoint2<N> for Point2<N> {
    fn to_3d(&self, z: N) -> Point3<N> {
        Point3::new(self.x, self.y, z)
    }

    // This uses an implicit origin vector and should be considered deprecated.
    fn negated_difference_and_angle_to(&self, other: Self) -> N {
        let diff = other - self;
        N::atan2(diff.y, diff.x)
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
    /// Convert this complex number (representing a 2D rotation) into a unit
    /// quaternion representing a 3D rotation around the z-axis.
    fn around_z_axis(&self) -> UnitQuaternion<f32>;
}

impl ExtendUnitComplex for UnitComplex<f32> {
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

impl ExtendRotation3 for Rotation3<f32> {
    fn from_unreal_angles(pitch: f32, yaw: f32, roll: f32) -> Rotation3<f32> {
        // Luckily, `nalgebra` and Unreal use the same rotation order. However, Unreal
        // negates the pitch and roll for some reason(?)
        Rotation3::from_euler_angles(-roll, -pitch, yaw)
    }

    fn to_unreal_angles(&self) -> (f32, f32, f32) {
        let (roll, pitch, yaw) = self.euler_angles();
        (-pitch, yaw, -roll)
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
    fn loc(&self) -> Point3<f32>;
    fn loc_2d(&self) -> Point2<f32>;
    fn rot(&self) -> Rotation3<f32>;
    fn quat(&self) -> UnitQuaternion<f32>;
    fn vel(&self) -> Vector3<f32>;
    fn vel_2d(&self) -> Vector2<f32>;
    fn ang_vel(&self) -> Vector3<f32>;
    /// A unit vector in the car's forward direction.
    fn forward_axis(&self) -> Unit<Vector3<f32>>;
    /// A unit vector in the car's forward direction in 2D space.
    fn forward_axis_2d(&self) -> Unit<Vector2<f32>>;
    /// A unit vector in the car's right direction.
    fn right_axis(&self) -> Unit<Vector3<f32>>;
    /// A unit vector towards the car's roof.
    fn roof_axis(&self) -> Unit<Vector3<f32>>;
}

impl ExtendPhysics for rlbot::ffi::Physics {
    fn loc(&self) -> Point3<f32> {
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

    fn vel_2d(&self) -> Vector2<f32> {
        Vector2::new(self.Velocity.X, self.Velocity.Y)
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

    fn roof_axis(&self) -> Unit<Vector3<f32>> {
        physics::car_roof_axis(self.quat())
    }
}

pub trait ExtendUnitQuaternion<N: Real> {
    fn xyzw(x: N, y: N, z: N, w: N) -> Self;
    fn to_2d(&self) -> UnitComplex<N>;
    fn project_2d(&self, axis: &Unit<Vector3<N>>) -> UnitComplex<N>;
}

impl<N: Real> ExtendUnitQuaternion<N> for UnitQuaternion<N> {
    fn xyzw(x: N, y: N, z: N, w: N) -> Self {
        UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
    }

    fn to_2d(&self) -> UnitComplex<N> {
        UnitComplex::new(self.scaled_axis().z)
    }

    fn project_2d(&self, axis: &Unit<Vector3<N>>) -> UnitComplex<N> {
        UnitComplex::new(self.scaled_axis().dot(axis))
    }
}

pub trait ExtendUnitVector3<N: Real> {
    fn to_2d(&self) -> Unit<Vector2<N>>;
    fn rotation_to(&self, other: &Self) -> UnitQuaternion<N>;
    fn angle_to(&self, other: &Self) -> N;
}

impl<N: Real> ExtendUnitVector3<N> for Unit<Vector3<N>> {
    fn to_2d(&self) -> Unit<Vector2<N>> {
        Unit::new_normalize(Vector2::new(self.x, self.y))
    }

    fn rotation_to(&self, other: &Self) -> UnitQuaternion<N> {
        UnitQuaternion::rotation_between_axis(self, other).unwrap()
    }

    fn angle_to(&self, other: &Self) -> N {
        match UnitQuaternion::rotation_between_axis(self, other) {
            Some(r) => r.angle(),
            None => N::pi(), // I'm only 90% sure this is 100% correct.
        }
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

pub trait ExtendRLBot {
    fn get_field_info(&self) -> Result<rlbot::ffi::FieldInfo, Box<dyn Error>>;
    fn quick_chat(
        &self,
        selection: rlbot::flat::QuickChatSelection,
        player_index: i32,
    ) -> Result<(), ()>;
}

impl ExtendRLBot for rlbot::RLBot {
    fn get_field_info(&self) -> Result<rlbot::ffi::FieldInfo, Box<dyn Error>> {
        let mut field_info = unsafe { mem::uninitialized() };
        self.update_field_info(&mut field_info)?;
        Ok(field_info)
    }

    fn quick_chat(
        &self,
        selection: rlbot::flat::QuickChatSelection,
        player_index: i32,
    ) -> Result<(), ()> {
        self.send_quick_chat(build_quick_chat(selection, player_index).finished_data())
            .map_err(|_| ())
    }
}

fn build_quick_chat(
    selection: rlbot::flat::QuickChatSelection,
    player_index: i32,
) -> flatbuffers::FlatBufferBuilder<'static> {
    let mut builder = flatbuffers::FlatBufferBuilder::new_with_capacity(32);
    let root = rlbot::flat::QuickChat::create(&mut builder, &rlbot::flat::QuickChatArgs {
        quickChatSelection: selection,
        playerIndex: player_index,
        teamOnly: false,
    });
    builder.finish(root, None);
    builder
}
