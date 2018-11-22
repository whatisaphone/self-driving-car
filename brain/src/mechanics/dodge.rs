use behavior::{Action, Behavior, Chain};
use mechanics::Yielder;
use rlbot;
use strategy::Context;

pub struct Dodge;

impl Dodge {
    pub fn new() -> Self {
        Dodge
    }
}

impl Behavior for Dodge {
    fn name(&self) -> &str {
        stringify!(Dodge)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if ctx.me().OnGround {
            ctx.eeg.log("[Dodge] can't dodge while on ground");
            return Action::Abort;
        }

        Action::call(Chain::new(
            self.priority(),
            vec![Box::new(Yielder::new(
                rlbot::ffi::PlayerInput {
                    Pitch: -1.0,
                    Jump: true,
                    ..Default::default()
                },
                0.05,
            ))],
        ))
    }
}
