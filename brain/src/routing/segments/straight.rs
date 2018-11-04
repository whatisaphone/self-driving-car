use arrayvec::ArrayVec;
use common::prelude::*;
use eeg::{color, Drawable};
use mechanics::simple_steer_towards;
use nalgebra::{Point2, Vector2};
use routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner};
use simulate::Car1D;
use strategy::Context;

#[derive(Clone)]
pub struct Straight {
    start: CarState2D,
    end_loc: Point2<f32>,
    end_vel: Vector2<f32>,
    end_boost: f32,
    duration: f32,
    mode: StraightMode,
}

/// This is a workaround for the lack of "arrive-at-time" behavior.
///
/// Most behaviors are able to control approach speed on a straightaway. If a
/// straight segment is followed by one of those behaviors, it's easier to just
/// defer to the behavior.
#[derive(Copy, Clone)]
pub enum StraightMode {
    /// Run the segment as fast as possible.
    Asap,
    /// Return immediately, depending on the subsequent behavior.
    Fake,
}

impl Straight {
    pub fn new(start: CarState2D, end_loc: Point2<f32>, end_chop: f32, mode: StraightMode) -> Self {
        const DT: f32 = 1.0 / 120.0;

        let start_to_end_dist = (end_loc - start.loc).norm();
        if start_to_end_dist < 1.0 {
            return Self::zero(start);
        }

        let mut sim = Car1D::new(start.vel.norm()).with_boost(start.boost);
        // Keep track of the simulated values so we can chop off an exact amount of time
        // once we reach the target distance.
        let mut sim_loc = ArrayVec::<[_; 1024]>::new();
        let mut sim_speed = ArrayVec::<[_; 1024]>::new();
        let mut sim_boost = ArrayVec::<[_; 1024]>::new();
        loop {
            sim_loc.push(sim.distance_traveled());
            sim_speed.push(sim.speed());
            sim_boost.push(sim.boost());
            if sim.distance_traveled() >= start_to_end_dist {
                break;
            }
            sim.step(DT, 1.0, true);
        }
        let end_frame = (sim_loc.len() - 1)
            .checked_sub((end_chop / DT) as usize)
            .unwrap_or(0);
        let sim_end_loc = sim_loc[end_frame];
        let sim_end_speed = sim_speed[end_frame];
        let sim_end_boost = sim_boost[end_frame];
        let end_loc = start.loc + (end_loc - start.loc).normalize() * sim_end_loc;
        if (end_loc - start.loc).norm() < 1.0 {
            return Self::zero(start);
        }
        let end_vel = (end_loc - start.loc).normalize() * sim_end_speed;

        Self {
            start,
            end_loc,
            end_vel,
            end_boost: sim_end_boost,
            duration: end_frame as f32 * DT,
            mode,
        }
    }

    fn zero(start: CarState2D) -> Self {
        Self {
            start: start.clone(),
            end_loc: start.loc,
            end_vel: start.vel,
            end_boost: start.boost,
            duration: 0.0,
            mode: StraightMode::Fake,
        }
    }
}

impl SegmentPlan for Straight {
    fn name(&self) -> &str {
        stringify!(Straight)
    }

    fn start(&self) -> CarState {
        self.start.to_3d()
    }

    fn end(&self) -> CarState {
        let result = CarState2D {
            loc: self.end_loc,
            rot: self.start.rot,
            vel: self.end_vel,
            boost: self.end_boost,
        }
        .to_3d();
        assert!(!result.vel.x.is_nan());
        result
    }

    fn duration(&self) -> f32 {
        self.duration
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(StraightRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg
            .draw(Drawable::Line(self.start.loc, self.end_loc, color::YELLOW));
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
            StraightMode::Asap => {} // continued below :)
        }

        let me = ctx.me();
        let me_loc = me.Physics.locp().to_2d();
        let start_to_end = self.plan.end_loc - self.plan.start.loc;
        let cur_dist = (me_loc - self.plan.start.loc).dot(&start_to_end.normalize());

        if cur_dist >= start_to_end.norm() {
            return SegmentRunAction::Success;
        }

        // Drive to a point slightly in front of us, so we "hug the line" and get back
        // on course quicker in case of any inaccuracies.
        let target_loc = self.plan.start.loc + start_to_end.normalize() * (cur_dist + 250.0);

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
