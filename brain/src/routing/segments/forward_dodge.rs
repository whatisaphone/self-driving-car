use crate::{
    behavior::{Action, Behavior, Chain, Priority},
    eeg::{color, Drawable},
    mechanics::Yielder,
    routing::models::{CarState, CarState2D, SegmentPlan, SegmentRunAction, SegmentRunner},
    strategy::Context,
};
use common::prelude::*;
use simulate::CarForwardDodge1D;

#[derive(Clone, new)]
pub struct ForwardDodge {
    start: CarState,
    dodge: CarForwardDodge1D,
}

impl SegmentPlan for ForwardDodge {
    fn name(&self) -> &str {
        stringify!(ForwardDodge)
    }

    fn start(&self) -> CarState {
        self.start.clone()
    }

    fn end(&self) -> CarState {
        assert!((self.start.vel.norm() - self.dodge.start_speed).abs() < 1.0);
        assert!(self.dodge.end_speed >= self.dodge.start_speed);
        let forward_axis = self.start.forward_axis_2d().unwrap();
        let vel =
            self.start.vel.to_2d() + forward_axis * (self.dodge.end_speed - self.dodge.start_speed);
        CarState2D {
            loc: self.start.loc.to_2d() + vel.normalize() * self.dodge.end_dist,
            rot: self.start.rot.to_2d(),
            vel,
            boost: self.start.boost,
        }
        .to_3d()
    }

    fn duration(&self) -> f32 {
        self.dodge.duration()
    }

    fn run(&self) -> Box<SegmentRunner> {
        Box::new(ForwardDodgeRunner::new(self.clone()))
    }

    fn draw(&self, ctx: &mut Context) {
        ctx.eeg.draw(Drawable::Line(
            self.start.loc.to_2d(),
            self.end().loc.to_2d(),
            color::GREEN,
        ));
    }
}

struct ForwardDodgeRunner {
    behavior: Box<Behavior>,
}

impl ForwardDodgeRunner {
    pub fn new(plan: ForwardDodge) -> Self {
        let behavior = Box::new(Chain::new(
            Priority::Idle,
            vec![
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Jump: true,
                        ..Default::default()
                    },
                    plan.dodge.jump_duration,
                )),
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        ..Default::default()
                    },
                    plan.dodge.wait_duration,
                )),
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Pitch: -1.0,
                        Jump: true,
                        ..Default::default()
                    },
                    6.0 / 120.0,
                )),
                Box::new(Yielder::new(
                    rlbot::ffi::PlayerInput {
                        ..Default::default()
                    },
                    plan.dodge.dodge_duration - 6.0 / 120.0,
                )),
            ],
        ));
        Self { behavior }
    }
}

impl SegmentRunner for ForwardDodgeRunner {
    fn execute(&mut self, ctx: &mut Context) -> SegmentRunAction {
        match self.behavior.execute2(ctx) {
            Action::Yield(i) => SegmentRunAction::Yield(i),
            Action::Call(_) => panic!("Action::Call not supported in SegmentRunner"),
            Action::Return => SegmentRunAction::Success,
            Action::Abort => SegmentRunAction::Failure,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point2, UnitComplex, Vector2};
    use simulate::CarForwardDodge;

    #[test]
    fn zero_vel() {
        let start = CarState2D {
            loc: Point2::origin(),
            rot: UnitComplex::identity(),
            vel: Vector2::zeros(),
            boost: 0.0,
        }
        .to_3d();
        let dodge = CarForwardDodge::calc_1d(0.0);
        let segment = ForwardDodge::new(start, dodge);
        let end = segment.end();
        assert!(end.loc.x >= 500.0);
        assert_eq!(end.vel.x, 500.0);
    }
}
