use crate::{
    behavior::movement::{simple_steer_towards, GetToFlatGround},
    eeg::{color, Drawable},
    routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
};
use common::{prelude::*, rl};
use nalgebra::{Point2, Vector2};
use nameof::name_of_type;
use simulate::Car1D;

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
        let start_to_end_dist = (end_loc - start.loc).norm();
        if start_to_end_dist < 0.1 {
            return Self::zero(start);
        }

        let mut sim = Car1D::new()
            .with_speed(start.vel.norm())
            .with_boost(start.boost);
        sim.advance_by_distance(start_to_end_dist, 1.0, true);

        // end_chop is the caller requesting we end the segment before reaching the
        // target.
        if end_chop != 0.0 {
            let duration = (sim.time() - end_chop).max(0.0);
            sim = Car1D::new()
                .with_speed(start.vel.norm())
                .with_boost(start.boost);
            sim.advance(duration, 1.0, true);
        }

        let sim_end_loc = sim.distance();
        let sim_end_speed = sim.speed();
        let sim_end_boost = sim.boost();

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
            duration: sim.time(),
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
        name_of_type!(Straight)
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
        let me_loc = me.Physics.loc_2d();
        let start_to_end = self.plan.end_loc - self.plan.start.loc;
        let cur_dist = (me_loc - self.plan.start.loc).dot(&start_to_end.normalize());

        if cur_dist >= start_to_end.norm() {
            return SegmentRunAction::Success;
        }

        if !GetToFlatGround::on_flat_ground(me) {
            ctx.eeg.log("[StraightRunner] Not on flat ground");
            return SegmentRunAction::Failure;
        }

        // Drive to a point slightly in front of us, so we "hug the line" and get back
        // on course quicker in case of any inaccuracies.
        let target_loc = self.plan.start.loc + start_to_end.normalize() * (cur_dist + 500.0);

        ctx.eeg
            .draw(Drawable::ghost_car_ground(target_loc, me.Physics.rot()));

        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: simple_steer_towards(&me.Physics, target_loc),
            Boost: me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED && me.Boost > 0,
            ..Default::default()
        })
    }
}
