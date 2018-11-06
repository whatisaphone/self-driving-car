use nalgebra::{Point2, Point3, Real, UnitQuaternion, Vector2, Vector3};
use physics;
use std::fmt::{self, Formatter};

pub trait PrettyPrint {
    type PrettyPrinter: fmt::Display;
    fn pretty(&self) -> Self::PrettyPrinter;
}

impl<N: Real> PrettyPrint for Vector2<N> {
    type PrettyPrinter = Vector2PrettyPrinter<N>;

    fn pretty(&self) -> Self::PrettyPrinter {
        Self::PrettyPrinter { data: *self }
    }
}

impl<N: Real> PrettyPrint for Point2<N> {
    type PrettyPrinter = Vector2PrettyPrinter<N>;

    fn pretty(&self) -> Self::PrettyPrinter {
        Self::PrettyPrinter { data: self.coords }
    }
}

pub struct Vector2PrettyPrinter<N: Real> {
    data: Vector2<N>,
}

impl<N: Real> fmt::Display for Vector2PrettyPrinter<N> {
    fn fmt(&self, f: &mut Formatter) -> Result<(), fmt::Error> {
        write!(f, "({:.0}, {:.0})", self.data.x, self.data.y)
    }
}

impl<N: Real> PrettyPrint for Vector3<N> {
    type PrettyPrinter = Vector3PrettyPrinter<N>;

    fn pretty(&self) -> Self::PrettyPrinter {
        Self::PrettyPrinter { data: *self }
    }
}

impl<N: Real> PrettyPrint for Point3<N> {
    type PrettyPrinter = Vector3PrettyPrinter<N>;

    fn pretty(&self) -> Self::PrettyPrinter {
        Self::PrettyPrinter { data: self.coords }
    }
}

pub struct Vector3PrettyPrinter<N: Real> {
    data: Vector3<N>,
}

impl<N: Real> fmt::Display for Vector3PrettyPrinter<N> {
    fn fmt(&self, f: &mut Formatter) -> Result<(), fmt::Error> {
        write!(
            f,
            "({:.0}, {:.0}, {:.0})",
            self.data.x, self.data.y, self.data.z,
        )
    }
}

impl PrettyPrint for UnitQuaternion<f32> {
    type PrettyPrinter = UnitQuaternionPrettyPrinter;

    fn pretty(&self) -> Self::PrettyPrinter {
        Self::PrettyPrinter { data: *self }
    }
}

pub struct UnitQuaternionPrettyPrinter {
    data: UnitQuaternion<f32>,
}

impl fmt::Display for UnitQuaternionPrettyPrinter {
    fn fmt(&self, f: &mut Formatter) -> Result<(), fmt::Error> {
        let forward = physics::car_forward_axis(self.data);
        write!(
            f,
            "forward is ({:.2}, {:.2}, {:.2})",
            forward.x, forward.y, forward.z,
        )
    }
}
