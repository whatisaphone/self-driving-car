use behavior::{Behavior, TepidHit};
use maneuvers::GetToFlatGround;
use strategy::{strategy::Strategy, Context};

#[derive(new)]
pub struct Dropshot;

impl Strategy for Dropshot {
    fn baseline(&mut self, ctx: &mut Context) -> Box<Behavior> {
        if !GetToFlatGround::on_flat_ground(ctx.packet) {
            return Box::new(GetToFlatGround::new());
        }

        Box::new(TepidHit::new())
    }

    fn interrupt(&mut self, _ctx: &mut Context, _current: &Behavior) -> Option<Box<Behavior>> {
        None
    }
}
