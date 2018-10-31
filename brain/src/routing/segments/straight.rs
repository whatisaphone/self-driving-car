use common::{physics::CAR_LOCAL_FORWARD_AXIS_2D, prelude::*};
use eeg::{color, Drawable};
use mechanics::simple_steer_towards;
use nalgebra::{Point2, Unit, UnitComplex, Vector2};
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use simulate::Car1D;
use strategy::Context;

#[derive(Clone)]
pub struct Straight {
    start_loc: Point2<f32>,
    start_vel: Vector2<f32>,
    start_boost: f32,
    end_loc: Point2<f32>,
    end_vel: Vector2<f32>,
    end_boost: f32,
    duration: f32,
    mode: StraightMode,
}

/// This is a workaround for the lack of "arrive-at-time" behavior.
#[derive(Clone)]
pub enum StraightMode {
    /// Run the behavior as fast as possible.
    Real,
    /// Return immediately, depending on the subsequent behavior.
    Fake,
}

impl Straight {
    pub fn new(
        start_loc: Point2<f32>,
        start_vel: Vector2<f32>,
        start_boost: f32,
        end_loc: Point2<f32>,
        mode: StraightMode,
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
            mode,
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
        Box::new(StraightRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg
            .draw(Drawable::Line(self.start_loc, self.end_loc, color::YELLOW));
    }
}

struct StraightRunner {
    plan: Straight,
}

impl StraightRunner {
    pub fn new(plan: Straight) -> Self {
        StraightRunner { plan }
    }
}

impl SegmentRunner for StraightRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        match self.plan.mode {
            StraightMode::Fake => {
                ctx.eeg
                    .log("[StraightRunner] stopping because mode is fake");
                return SegmentRunAction::Success;
            }
            StraightMode::Real => {} // continued below :)
        }

        let me = ctx.me();
        let me_loc = me.Physics.locp().to_2d();
        let start_to_end = self.plan.end_loc - self.plan.start_loc;
        let cur_dist = (me_loc - self.plan.start_loc).dot(&start_to_end.normalize());

        if cur_dist >= start_to_end.norm() {
            return SegmentRunAction::Success;
        }

        // Drive to a point slightly in front of us, so we "hug the line" and get back
        // on course quicker in case of any inaccuracies.
        let target_loc = self.plan.start_loc + start_to_end.normalize() * (cur_dist + 250.0);

        ctx.eeg.draw(Drawable::ghost_car_ground(
            target_loc.coords,
            me.Physics.rot(),
        ));

        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: simple_steer_towards(&me.Physics, target_loc.coords),
            Boost: true,
            ..Default::default()
        })
    }
}
