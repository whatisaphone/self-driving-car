use crate::{
    behavior::movement::GetToFlatGround,
    eeg::{color, Drawable},
    routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
};
use nalgebra::Vector2;
use nameof::name_of_type;
use simulate::CarPowerslideTurnBlueprint;

#[derive(Clone)]
pub struct PowerslideTurn {
    blueprint: CarPowerslideTurnBlueprint,
    boost: f32,
}

impl PowerslideTurn {
    pub fn new(blueprint: CarPowerslideTurnBlueprint, boost: f32) -> Self {
        Self { blueprint, boost }
    }
}

impl SegmentPlan for PowerslideTurn {
    fn name(&self) -> &str {
        name_of_type!(PowerslideTurn)
    }

    fn start(&self) -> CarState {
        CarState2D {
            loc: self.blueprint.start_loc,
            rot: self.blueprint.start_rot,
            vel: self.blueprint.start_vel,
            boost: self.boost,
        }
        .to_3d()
    }

    fn end(&self) -> CarState {
        CarState2D {
            loc: self.blueprint.end_loc,
            rot: self.blueprint.end_rot,
            // Subsequent segments will likely expect us to be fully recovered and not skidding so
            // let's uh, yeah, we can do that, no problem! You're never skidding while standing
            // still.
            vel: Vector2::zeros(),
            boost: self.boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        self.blueprint.duration
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(PowerslideTurnRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg.draw(Drawable::Line(
            self.blueprint.start_loc,
            self.blueprint.end_loc,
            color::RED,
        ));
    }
}

struct PowerslideTurnRunner {
    plan: PowerslideTurn,
    start_time: Option<f32>,
}

impl PowerslideTurnRunner {
    fn new(plan: PowerslideTurn) -> Self {
        Self {
            plan,
            start_time: None,
        }
    }
}

impl SegmentRunner for PowerslideTurnRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        let now = ctx.packet.GameInfo.TimeSeconds;
        let start_time = *self.start_time.get_or_insert(now);
        let elapsed = now - start_time;
        // Crudely account for recovery time with a fudge factor
        let duration = self.plan.blueprint.duration * 0.5;
        if elapsed >= duration {
            return SegmentRunAction::Success;
        }

        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            ctx.eeg.log("[PowerslideTurnRunner] Not on flat ground");
            return SegmentRunAction::Failure;
        }

        SegmentRunAction::Yield(rlbot::ffi::PlayerInput {
            Throttle: self.plan.blueprint.throttle,
            Steer: self.plan.blueprint.steer,
            Handbrake: true,
            ..Default::default()
        })
    }
}
