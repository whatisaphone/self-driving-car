use crate::{
    behavior::{movement::GetToFlatGround, offense::TepidHit},
    strategy::{strategy::Strategy, Behavior, Context},
};
use derive_new::new;

#[derive(new)]
pub struct Dropshot;

impl Strategy for Dropshot {
    fn baseline(&mut self, ctx: &mut Context<'_>) -> Box<dyn Behavior> {
        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            return Box::new(GetToFlatGround::new());
        }

        Box::new(TepidHit::new())
    }

    fn interrupt(
        &mut self,
        _ctx: &mut Context<'_>,
        _current: &dyn Behavior,
    ) -> Option<Box<dyn Behavior>> {
        None
    }
}
