use common::{physics::CAR_LOCAL_FORWARD_AXIS_2D, prelude::*};
use eeg::{color, Drawable};
use maneuvers::GetToFlatGround;
use nalgebra::{Point2, Unit, UnitComplex, Vector2};
use rlbot;
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use std::f32::consts::PI;
use strategy::Context;

#[derive(Clone)]
pub struct SimpleArc {
    center: Point2<f32>,
    radius: f32,
    start_loc: Point2<f32>,
    start_vel: Vector2<f32>,
    start_boost: f32,
    sweep: f32,
}

pub enum SimpleArcError {
    VelocityTooLow,
    WrongGeometry,
}

impl SimpleArcError {
    pub fn to_str(&self) -> &'static str {
        match self {
            SimpleArcError::VelocityTooLow => stringify!(SimpleArcError::VelocityTooLow),
            SimpleArcError::WrongGeometry => stringify!(SimpleArcError::WrongGeometry),
        }
    }
}

impl SimpleArc {
    pub fn new(
        center: Point2<f32>,
        radius: f32,
        start_loc: Point2<f32>,
        start_vel: Vector2<f32>,
        start_boost: f32,
        end_loc: Point2<f32>,
    ) -> Result<Self, SimpleArcError> {
        // This assumes a constant speed and will estimate a ridiculous duration if the
        // velocity is too low.
        if start_vel.norm() < 100.0 {
            return Err(SimpleArcError::VelocityTooLow);
        }

        // Assert that both radii are the same length.
        if ((start_loc - center).norm() - (end_loc - center).norm()).abs() >= 1.0 {
            return Err(SimpleArcError::WrongGeometry);
        }

        // Compare the velocity vector to the circle's radius. Since we're starting
        // along a tangent, the angle to the center will be either -90° or 90°.
        let clockwise = start_vel.rotation_to(start_loc - center).angle() < 0.0;

        // Go the long way around the circle (more than 180°) if necessary. This avoids
        // an impossible route with discontinuous reversals at each tangent.
        let sweep = (start_loc - center).rotation_to(end_loc - center).angle();
        let sweep = if clockwise && sweep < 0.0 {
            sweep + 2.0 * PI
        } else if !clockwise && sweep >= 0.0 {
            sweep - 2.0 * PI
        } else {
            sweep
        };

        Ok(Self {
            center,
            radius,
            start_loc,
            start_vel,
            start_boost,
            sweep,
        })
    }

    /// Calculate a rotation of the given angle in this plan's direction.
    fn sweep_by_angle(&self, angle: f32) -> f32 {
        angle * self.sweep.signum()
    }

    /// Calculate the angle between the two points, traveling in this plan's
    /// direction.
    fn sweep_between(&self, start_loc: Point2<f32>, end_loc: Point2<f32>) -> f32 {
        let result = (start_loc - self.center)
            .rotation_to(end_loc - self.center)
            .angle();
        if result < 0.0 && self.sweep >= 0.0 {
            result + 2.0 * PI
        } else if result > 0.0 && self.sweep < 0.0 {
            result - 2.0 * PI
        } else {
            result
        }
    }

    fn start_rot(&self) -> UnitComplex<f32> {
        let dir = Unit::new_normalize(self.start_vel);
        CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&dir)
    }

    fn end_loc(&self) -> Point2<f32> {
        self.center + UnitComplex::new(self.sweep) * (self.start_loc - self.center)
    }

    fn end_rot(&self) -> UnitComplex<f32> {
        let dir = Unit::new_normalize(self.end_vel());
        CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&dir)
    }

    fn end_vel(&self) -> Vector2<f32> {
        UnitComplex::new(self.sweep) * self.start_vel
    }
}

impl SegmentPlan for SimpleArc {
    fn name(&self) -> &str {
        stringify!(SimpleArc)
    }

    fn start(&self) -> CarState {
        CarState2D {
            loc: self.start_loc,
            rot: self.start_rot(),
            vel: self.start_vel,
            boost: self.start_boost,
        }
        .to_3d()
    }

    fn end(&self) -> CarState {
        CarState2D {
            loc: self.end_loc(),
            rot: self.end_rot(),
            vel: self.end_vel(),
            boost: self.start_boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        self.radius * self.sweep.abs() / self.start_vel.norm()
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(SimpleArcRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        let theta1 = Vector2::x()
            .rotation_to(self.start_loc - self.center)
            .angle();
        let theta2 = theta1 + self.sweep;
        ctx.eeg.draw(Drawable::Arc(
            self.center,
            self.radius,
            theta1.min(theta2),
            theta1.max(theta2),
            color::YELLOW,
        ));
    }
}

struct SimpleArcRunner {
    plan: SimpleArc,
}

impl SimpleArcRunner {
    fn new(plan: SimpleArc) -> Self {
        Self { plan }
    }

    fn calculate_ahead_loc(&self, loc: Point2<f32>, angle: f32) -> Point2<f32> {
        let center_to_loc = loc - self.plan.center;
        let center_to_ahead = self.plan.sweep_by_angle(angle) * center_to_loc;
        self.plan.center + center_to_ahead.normalize() * self.plan.radius
    }
}

impl SegmentRunner for SimpleArcRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let me = ctx.me();
        let car_loc = me.Physics.locp().to_2d();
        let car_forward_axis = me.Physics.forward_axis().to_2d();

        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            ctx.eeg.log("[SimpleArcRunner] Not on flat ground");
            return SegmentRunAction::Failure;
        }

        // Check if we're finished.
        let swept = self.plan.sweep_between(self.plan.start_loc, car_loc);
        if swept.abs() >= self.plan.sweep.abs() {
            return SegmentRunAction::Success;
        }

        let target_loc = self.calculate_ahead_loc(car_loc, 15.0_f32.to_radians());

        ctx.eeg.draw(Drawable::ghost_car_ground(
            target_loc.coords,
            me.Physics.rot(),
        ));

        let rotation = car_forward_axis.unwrap().rotation_to(target_loc - car_loc);
        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: rotation.angle().max(-1.0).min(1.0),
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Point2, Vector2, Vector3};
    use routing::{segments::SimpleArc, test::segment_plan_tester};

    #[test]
    #[ignore(note = "This is a demo, not a test")]
    fn simple_arc_demo() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                car_loc: Vector3::new(1000.0, 0.0, 17.01),
                car_vel: Vector3::new(100.0, 0.0, 0.0),
                ..Default::default()
            })
            .behavior(segment_plan_tester(
                SimpleArc::new(
                    Point2::origin(),
                    1000.0,
                    Point2::new(1000.0, 0.0),
                    Vector2::new(0.0, 100.0),
                    0.0,
                    Point2::new(0.0, 1000.0),
                )
                .ok()
                .unwrap(),
            ))
            .run();
        test.sleep_millis(10_000);
    }
}
