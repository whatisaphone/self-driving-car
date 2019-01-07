use crate::{
    routing::models::{CarState, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
    utils::geometry::flattener::Flattener,
};
use common::{prelude::*, rl};
use nalgebra::{Point2, Point3, Vector2};
use nameof::name_of_type;
use simulate::Car1D;

#[derive(Clone)]
pub struct WallStraight {
    start: CarState,
    target_loc: Point3<f32>,
    start_to_flat: Flattener,
    target_to_flat: Flattener,
    flat_end_loc: Point2<f32>,
    flat_end_vel: Vector2<f32>,
    duration: f32,
}

impl WallStraight {
    pub fn new(
        start: CarState,
        target_loc: Point3<f32>,
        start_to_flat: Flattener,
        target_to_flat: Flattener,
    ) -> Self {
        let flat_start = start.flatten(&start_to_flat);
        let flat_target_loc = target_to_flat * target_loc;
        let flat_target_dist = (flat_target_loc - flat_start.loc).norm();

        let mut sim = Car1D::new()
            .with_speed(flat_start.vel.norm())
            .with_boost(flat_start.boost);

        // HACK: set boost = false to limit the simulated range since driving on the
        // wall is slower.
        sim.advance_by_distance(flat_target_dist, 1.0, false);

        let flat_dir = (flat_target_loc - flat_start.loc).normalize();
        let flat_end_loc = flat_start.loc + flat_dir * sim.distance();
        let flat_end_vel = flat_dir * sim.speed();

        Self {
            start,
            target_loc,
            start_to_flat,
            target_to_flat,
            flat_end_loc,
            flat_end_vel,
            duration: sim.time(),
        }
    }
}

impl SegmentPlan for WallStraight {
    fn name(&self) -> &str {
        name_of_type!(WallStraight)
    }

    fn start(&self) -> CarState {
        self.start.clone()
    }

    fn end(&self) -> CarState {
        let flat_to_target = self.target_to_flat.inverse();
        CarState {
            loc: flat_to_target * self.flat_end_loc.to_3d(rl::OCTANE_NEUTRAL_Z),
            rot: flat_to_target.rotation * (self.start_to_flat * self.start.rot).around_z_axis(),
            vel: flat_to_target * self.flat_end_vel.to_3d(0.0),
            boost: 0.0,
        }
    }

    fn duration(&self) -> f32 {
        self.duration
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(WallStraightRunner)
    }

    fn draw(&self, _ctx: &mut Context) {}
}

struct WallStraightRunner;

impl SegmentRunner for WallStraightRunner {
    fn name(&self) -> &str {
        name_of_type!(WallStraightRunner)
    }

    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let (_ctx, eeg) = ctx.split();

        // Assume the subsequent action will do this for us.
        eeg.log(self.name(), "blindly succeeding, somebody pick up my slack");

        SegmentRunAction::Success
    }
}
