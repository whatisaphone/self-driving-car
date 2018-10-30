use common::{physics::CAR_LOCAL_FORWARD_AXIS_2D, prelude::*};
use eeg::{color, Drawable};
use nalgebra::{Point2, Unit, UnitComplex, Vector2};
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use simulate::Car1D;
use strategy::Context;

pub struct Straight {
    start_loc: Point2<f32>,
    start_vel: Vector2<f32>,
    start_boost: f32,
    end_loc: Point2<f32>,
    end_vel: Vector2<f32>,
    end_boost: f32,
    duration: f32,
}

impl Straight {
    pub fn new(
        start_loc: Point2<f32>,
        start_vel: Vector2<f32>,
        start_boost: f32,
        end_loc: Point2<f32>,
    ) -> Self {
        let mut car = Car1D::new(start_vel.norm()).with_boost(start_boost);
        let total_dist = (end_loc - start_loc).norm();
        loop {
            car.step(1.0 / 120.0, 1.0, true);
            if car.distance_traveled() >= total_dist {
                break;
            }
        }
        let end_vel = (end_loc - start_loc).normalize() * car.speed();

        Self {
            start_loc,
            start_vel,
            start_boost,
            end_loc,
            end_vel,
            end_boost: car.boost(),
            duration: car.time(),
        }
    }

    fn rot(&self) -> UnitComplex<f32> {
        let dir = Unit::new_normalize(self.end_loc - self.start_loc);
        CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&dir)
    }
}

impl SegmentPlan for Straight {
    fn start(&self) -> CarState {
        CarState2D {
            loc: self.start_loc,
            rot: self.rot(),
            vel: self.start_vel,
            boost: self.start_boost,
        }
        .to_3d()
    }

    fn end(&self) -> CarState {
        CarState2D {
            loc: self.end_loc,
            rot: self.rot(),
            vel: self.end_vel,
            boost: self.end_boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        self.duration
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(StraightRunner::new())
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg
            .draw(Drawable::Line(self.start_loc, self.end_loc, color::YELLOW));
    }
}

struct StraightRunner;

impl StraightRunner {
    pub fn new() -> Self {
        StraightRunner
    }
}

impl SegmentRunner for StraightRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        ctx.eeg
            .log("[StraightRunner] I assume this will be handled somewhere else");
        SegmentRunAction::Success
    }
}
