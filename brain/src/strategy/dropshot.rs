use crate::{
    behavior::{movement::GetToFlatGround, offense::TepidHit, Behavior},
    strategy::{strategy::Strategy, Context},
};
use derive_new::new;

#[derive(new)]
pub struct Dropshot;

impl Strategy for Dropshot {
    fn baseline(&mut self, ctx: &mut Context) -> Box<Behavior> {
        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            return Box::new(GetToFlatGround::new());
        }

        Box::new(TepidHit::new())
    }

    fn interrupt(&mut self, _ctx: &mut Context, _current: &Behavior) -> Option<Box<Behavior>> {
        None
    }
}
