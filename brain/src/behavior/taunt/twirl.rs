use crate::strategy::{Action, Behavior, Context, Priority};
use common::{prelude::*, rl};
use dom::get_pitch_yaw_roll;
use nalgebra::Vector3;
use nameof::name_of_type;

pub struct Twirl {
    force_boost: bool,
}

impl Twirl {
    pub fn new(force_boost: bool) -> Self {
        Self { force_boost }
    }
}

impl Behavior for Twirl {
    fn name(&self) -> &str {
        name_of_type!(Twirl)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let car = ctx.me();
        let car_forward_axis = car.Physics.forward_axis();

        let forward = car_forward_axis
            .to_2d()
            .to_3d()
            .rotation_to(&Vector3::z_axis())
            .powf(0.6666667)
            * car_forward_axis.to_2d().to_3d();
        let (_pitch, yaw, _roll) = get_pitch_yaw_roll(car, forward, Vector3::z_axis());

        let apex = kinematic_apex(car.Physics.loc().z, car.Physics.vel().z, rl::GRAVITY);
        let too_low = apex < 75.0;

        Action::Yield(common::halfway_house::PlayerInput {
            Pitch: -0.3333333,
            Yaw: yaw,
            Roll: 0.6666667,
            Boost: too_low || self.force_boost,
            ..Default::default()
        })
    }
}

fn kinematic_apex(d_0: f32, v_0: f32, a: f32) -> f32 {
    let v_f = 0.0;
    let t = (v_f - v_0) / a;
    if t >= 0.0 {
        let d = v_0 * t + 0.5 * a * t * t;
        d_0 + d
    } else {
        let boost_accel_guess = 900.0;
        let a = a + boost_accel_guess;
        let t = (v_f - v_0) / a;
        let d = v_0 * t + 0.5 * a * t * t;
        d_0 + d
    }
}
