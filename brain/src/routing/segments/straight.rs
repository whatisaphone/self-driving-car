use common::{
    ext::{ExtendPhysics, ExtendUnitVector2},
    physics::CAR_LOCAL_FORWARD_AXIS_2D,
};
use eeg::{color, Drawable};
use mechanics::simple_steer_towards;
use nalgebra::{Point2, Unit, UnitComplex, Vector2};
use rlbot;
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use simulate::Car1D;
use strategy::Context;
use utils::geometry::ExtendPoint3;

#[derive(Clone)]
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
        let mut car = Car1D::new(start_vel.norm()).with_boost_float(start_boost);
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

    fn truncate_to_duration(&self, duration: f32) -> Box<SegmentPlan> {
        let mut car = Car1D::new(self.start_vel.norm()).with_boost_float(self.start_boost);
        let total_dist = (self.end_loc - self.start_loc).norm();
        loop {
            car.step(1.0 / 120.0, 1.0, true);
            if car.distance_traveled() >= total_dist {
                break;
            }
            if car.time() >= duration {
                break;
            }
        }

        Box::new(Self {
            start_loc: self.start_loc,
            start_vel: self.start_vel,
            start_boost: self.start_boost,
            end_loc: self.start_loc
                + (self.end_loc - self.start_loc).normalize() * car.distance_traveled(),
            end_vel: (self.end_loc - self.start_loc).normalize() * car.speed(),
            end_boost: car.boost(),
            duration: car.time(),
        })
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
        Self { plan }
    }
}

impl SegmentRunner for StraightRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let me = ctx.me();
        let me_loc = me.Physics.locp().to_2d();
        let cur_dist = (me_loc - self.plan.start_loc)
            .dot(&(self.plan.end_loc - self.plan.start_loc).normalize());

        if cur_dist >= (self.plan.end_loc - self.plan.start_loc).norm() {
            return SegmentRunAction::Success;
        }

        let target_loc = self.plan.start_loc
            + (self.plan.end_loc - self.plan.start_loc).normalize() * (cur_dist + 250.0);

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
