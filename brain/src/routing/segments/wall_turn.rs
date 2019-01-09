use crate::{
    routing::models::{CarState, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
    utils::geometry::{flattener::Flattener, Plane},
};
use common::{prelude::*, rl};
use nalgebra::{Point2, Point3, UnitComplex};
use nameof::name_of_type;
use std::f32::consts::PI;

#[derive(Clone)]
pub struct WallTurn {
    start: CarState,
    surface: Plane,
    flattener: Flattener,
    flat_center: Point2<f32>,
    radius: f32,
    flat_target_loc: Point2<f32>,
    sweep: f32,
    face_loc: Point3<f32>,
}

impl WallTurn {
    pub fn new(
        start: CarState,
        surface: Plane,
        flattener: Flattener,
        flat_center: Point2<f32>,
        radius: f32,
        flat_target_loc: Point2<f32>,
        face_loc: Point3<f32>,
    ) -> Self {
        let flat_start_loc = flattener * start.loc;
        let sweep = (flat_start_loc - flat_center).angle_to(&(flat_target_loc - flat_center));

        Self {
            start,
            surface,
            flattener,
            flat_center,
            radius,
            flat_target_loc,
            sweep,
            face_loc,
        }
    }

    /// Calculate the angle between this plan's start point and the given end
    /// point, traveling in this plan's direction.
    fn sweep_to(&self, end_loc: Point2<f32>) -> f32 {
        let flat_start_loc = self.flattener * self.start.loc;
        let result = (flat_start_loc - self.flat_center).angle_to(&(end_loc - self.flat_center));
        if result < 0.0 && self.sweep >= 0.0 {
            result + 2.0 * PI
        } else if result > 0.0 && self.sweep < 0.0 {
            result - 2.0 * PI
        } else {
            result
        }
    }
}

impl SegmentPlan for WallTurn {
    fn name(&self) -> &str {
        name_of_type!(WallTurn)
    }

    fn start(&self) -> CarState {
        self.start.clone()
    }

    fn end(&self) -> CarState {
        let extrude = self.flattener.inverse();
        let sweep = UnitComplex::new(self.sweep);
        CarState {
            loc: extrude * self.flat_target_loc.to_3d(rl::OCTANE_NEUTRAL_Z),
            rot: extrude.rotation * (sweep * (self.flattener * self.start.rot)).around_z_axis(),
            vel: extrude * (sweep * (self.flattener * self.start.vel)).to_3d(0.0),
            boost: self.start.boost,
        }
    }

    fn duration(&self) -> f32 {
        let assume_speed = f32::max(self.start.vel.norm(), 800.0) * 2.0;
        self.radius * self.sweep.abs() / assume_speed
    }

    fn run(&self) -> Box<dyn SegmentRunner> {
        Box::new(WallTurnRunner::new(self.clone()))
    }

    fn draw(&self, _ctx: &mut Context<'_>) {}
}

struct WallTurnRunner {
    plan: WallTurn,
}

impl WallTurnRunner {
    pub fn new(plan: WallTurn) -> Self {
        Self { plan }
    }
}

impl SegmentRunner for WallTurnRunner {
    fn name(&self) -> &str {
        name_of_type!(WallTurnRunner)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> SegmentRunAction {
        let me = ctx.me();
        let me_flat_loc = self.plan.flattener * me.Physics.loc();
        let me_flat_forward = self.plan.flattener * me.Physics.forward_axis();

        if !me.OnGround {
            ctx.eeg.log(self.name(), "not on ground");
            return SegmentRunAction::Failure;
        }

        // Check two end conditions to decrease the chances that silly things happen.

        let flat_face_dir = self.plan.flattener * self.plan.face_loc - me_flat_loc;
        let steer = me_flat_forward.angle_to(&flat_face_dir);
        if steer.abs() < PI / 15.0 {
            return SegmentRunAction::Success;
        }

        let swept = self.plan.sweep_to(me_flat_loc);
        if swept.abs() >= self.plan.sweep.abs() - 3.0_f32.to_radians() {
            return SegmentRunAction::Success;
        }

        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: steer.signum(),
            ..Default::default()
        })
    }
}
