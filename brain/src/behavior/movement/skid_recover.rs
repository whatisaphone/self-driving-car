use crate::{
    behavior::movement::get_to_flat_ground::GetToFlatGround,
    eeg::Drawable,
    strategy::{Action, Behavior, Context},
};
use common::{physics::CAR_LOCAL_FORWARD_AXIS_2D, prelude::*, AngularVelocity};
use derive_new::new;
use nalgebra::{Point2, UnitComplex};
use nameof::name_of_type;

#[derive(new)]
pub struct SkidRecover {
    target_loc: Point2<f32>,
}

impl Behavior for SkidRecover {
    fn name(&self) -> &str {
        name_of_type!(SkidRecover)
    }

    fn execute(&mut self, ctx: &mut Context) -> Action {
        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            ctx.eeg.log(self.name(), "must be on flat ground");
            return Action::Abort;
        }

        let me = ctx.me();
        let me_rot = me.Physics.quat().to_2d();
        let me_ang_vel = me.Physics.ang_vel().z;
        let me_to_target = self.target_loc - me.Physics.loc_2d();

        let target_rot = CAR_LOCAL_FORWARD_AXIS_2D.rotation_to(&me_to_target.to_axis());
        // Since we're skidding, aim towards where we will be a bit in the future.
        // Otherwise we'll overshoot.
        let future_rot = target_rot * UnitComplex::new(me_ang_vel * 0.25);
        let steer = me_rot.rotation_to(&future_rot).angle().max(-1.0).min(1.0);

        ctx.eeg.draw(Drawable::ghost_car_ground(
            self.target_loc,
            target_rot.around_z_axis().to_rotation_matrix(),
        ));
        ctx.eeg.print_angle("rot", me_rot.angle());
        ctx.eeg.print_angle("target_rot", target_rot.angle());
        ctx.eeg.print_value("ang_vel", AngularVelocity(me_ang_vel));
        ctx.eeg.print_angle("future_rot", future_rot.angle());

        Action::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: steer,
            ..Default::default()
        })
    }
}
