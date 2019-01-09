use crate::{
    behavior::{higher_order::Chain, movement::yielder::Yielder},
    strategy::{Action, Behavior, Context},
};
use nalgebra::UnitComplex;
use nameof::name_of_type;

pub struct Dodge {
    angle: UnitComplex<f32>,
}

impl Dodge {
    pub fn new() -> Self {
        Self {
            angle: UnitComplex::identity(),
        }
    }

    /// The angle of the dodge, where 0° means straight forward.
    pub fn angle(mut self, angle: UnitComplex<f32>) -> Self {
        self.angle = angle;
        self
    }
}

impl Behavior for Dodge {
    fn name(&self) -> &str {
        name_of_type!(Dodge)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if ctx.me().OnGround {
            ctx.eeg.log(self.name(), "can't dodge while on ground");
            return Action::Abort;
        }

        let pitch = -self.angle.cos_angle();
        let yaw = self.angle.sin_angle();

        Action::call(Chain::new(self.priority(), vec![Box::new(Yielder::new(
            rlbot::ffi::PlayerInput {
                Pitch: pitch,
                Yaw: yaw,
                Jump: true,
                ..Default::default()
            },
            0.05,
        ))]))
    }
}
