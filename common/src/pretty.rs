use crate::physics;
use nalgebra::{Point2, Point3, Rotation3, Unit, UnitQuaternion, Vector2, Vector3};
use std::fmt::{self, Formatter};

pub trait PrettyPrint {
    type PrettyPrinter: fmt::Display;
    fn pretty(&self) -> Self::PrettyPrinter;
}

macro_rules! delegate {
    ($type:ty $(,)*) => {
        impl PrettyPrint for $type {
            type PrettyPrinter = Self;

            fn pretty(&self) -> Self::PrettyPrinter {
                self.clone()
            }
        }
    };
}

macro_rules! inherent {
    ($printer:ident, $type:ty, $fmt:expr $(,)*) => {
        inherent!($printer, $type, $fmt, |x| x);
    };

    ($printer:ident, $type:ty, $fmt:expr, | $x:pat | ($($map:expr),+ $(,)*) $(,)*) => {
        impl PrettyPrint for $type {
            type PrettyPrinter = $printer;

            fn pretty(&self) -> Self::PrettyPrinter {
                $printer(*self)
            }
        }

        pub struct $printer($type);

        impl fmt::Display for $printer {
            fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), fmt::Error> {
                let $x = self.0;
                write!(f, $fmt, $($map),+)
            }
        }
    };

    ($printer:ident, $type:ty, $fmt:expr, | $x:pat | $map:expr $(,)*) => {
        inherent!($printer, $type, $fmt, |$x| ($map));
    };
}

macro_rules! wrap {
    ($name:ident, $type:ty, $fmt:expr $(,)*) => {
        wrap!($name, $type, $fmt, |x| x);
    };

    ($name:ident, $type:ty, $fmt:expr, | $x:pat | $map:expr $(,)*) => {
        #[derive(Copy, Clone)]
        pub struct $name(pub $type);

        impl PrettyPrint for $name {
            type PrettyPrinter = Self;

            fn pretty(&self) -> Self::PrettyPrinter {
                *self
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), fmt::Error> {
                let $x = self.0;
                let s = $map;
                write!(f, $fmt, s)
            }
        }
    };
}

delegate!(bool);
delegate!(String);

inherent!(P2PP, Point2<f32>, "({:.0}, {:.0})", |v| (v.x, v.y));
inherent!(V2PP, Vector2<f32>, "({:.0}, {:.0})", |v| (v.x, v.y));
inherent!(P3PP, Point3<f32>, "({:.0}, {:.0}, {:.0})", |v| (
    v.x, v.y, v.z,
));
inherent!(V3PP, Vector3<f32>, "({:.0}, {:.0}, {:.0})", |v| (
    v.x, v.y, v.z,
));
inherent!(UV3PP, Unit<Vector3<f32>>, "({:.2}, {:.2}, {:.2})", |v| (
    v.x, v.y, v.z,
));
inherent!(UQPP, UnitQuaternion<f32>, "@{}", |x| {
    physics::car_forward_axis(x).pretty()
});
inherent!(R3PP, Rotation3<f32>, "{}", |x| {
    UnitQuaternion::from_rotation_matrix(&x).pretty()
});

wrap!(Time, f32, "{:.2}");
wrap!(Coordinate, f32, "{:.0}");
wrap!(Distance, f32, "{:.0}");
wrap!(Angle, f32, "{:.0}°", |x| x.to_degrees());
wrap!(AngularVelocity, f32, "{:.0}°/s", |x| x.to_degrees());
wrap!(ControllerInput, f32, "{:.2}");
