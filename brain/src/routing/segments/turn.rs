use common::prelude::*;
use eeg::{color, Drawable};
use maneuvers::drive_towards;
use nalgebra::{Point2, UnitComplex, Vector2};
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use strategy::Context;
use utils::geometry::ExtendPoint3;

#[derive(Clone)]
pub struct Turn {
    start: CarState2D,
    target_loc: Point2<f32>,
    center: Point2<f32>,
    radius: f32,
    sweep: f32,
}

impl Turn {
    pub fn new(
        start: CarState,
        target_loc: Point2<f32>,
        center: Point2<f32>,
        radius: f32,
        projected_end_loc: Point2<f32>,
    ) -> Self {
        let start = CarState2D {
            loc: start.loc.to_2d(),
            rot: start.rot.to_2d(),
            vel: start.vel.to_2d(),
            boost: 0.0,
        };

        let sweep = (start.loc - center)
            .rotation_to(projected_end_loc - center)
            .angle();

        Self {
            start,
            target_loc,
            center,
            radius,
            sweep,
        }
    }
}

impl SegmentPlan for Turn {
    fn start(&self) -> CarState {
        self.start.to_3d()
    }

    fn end(&self) -> CarState {
        let sweep = UnitComplex::new(self.sweep);
        CarState2D {
            loc: self.center + sweep * (self.start.loc - self.center),
            rot: sweep * self.start.rot,
            vel: sweep * self.start.vel,
            boost: self.start.boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        let assume_speed = f32::max(self.start.vel.norm(), 800.0);
        self.radius * self.sweep.abs() / assume_speed
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(Turner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        let theta1 = Vector2::x()
            .rotation_to(self.start.loc - self.center)
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

struct Turner {
    plan: Turn,
}

impl Turner {
    pub fn new(plan: Turn) -> Self {
        Self { plan }
    }
}

impl SegmentRunner for Turner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let me = ctx.me();
        let me_forward = me.Physics.forward_axis_2d();
        let steer = me_forward.rotation_to(&(self.plan.target_loc - me.Physics.loc_2d()).to_axis());
        if steer.angle() < 5.0_f32.to_radians() {
            return SegmentRunAction::Success;
        }

        SegmentRunAction::Yield(drive_towards(
            ctx.packet,
            ctx.eeg,
            self.plan.target_loc.coords,
        ))
    }
}
