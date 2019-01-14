use crate::{
    behavior::movement::Yielder,
    strategy::{Action, Behavior, Context, Priority},
};
use common::prelude::*;
use derive_new::new;
use nalgebra::Vector3;
use nameof::name_of_type;
use std::f32::consts::PI;

#[derive(new)]
pub struct TurtleSpin;

impl Behavior for TurtleSpin {
    fn name(&self) -> &str {
        name_of_type!(TurtleSpin)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if ctx.me().OnGround {
            return Action::tail_call(
                Yielder::new(
                    rlbot::ffi::PlayerInput {
                        Roll: 1.0,
                        Jump: true,
                        Boost: ctx.me().Boost > 0,
                        ..Default::default()
                    },
                    0.2,
                )
                .priority(self.priority()),
            );
        }

        if should_air_roll_upside_down(ctx) {
            return Action::Yield(rlbot::ffi::PlayerInput {
                Roll: 1.0,
                Boost: ctx.me().Boost > 0,
                ..Default::default()
            });
        }

        if !ctx.me().DoubleJumped {
            return Action::Yield(rlbot::ffi::PlayerInput {
                Jump: true,
                ..Default::default()
            });
        }

        Action::Yield(rlbot::ffi::PlayerInput {
            Yaw: 1.0,
            Jump: !ctx.me().DoubleJumped,
            ..Default::default()
        })
    }
}

fn should_air_roll_upside_down(ctx: &mut Context<'_>) -> bool {
    // If we're spinning up the wall, let it happen naturally.
    let closest_plane = ctx.game.pitch().closest_plane(&ctx.me().Physics.loc());
    if closest_plane.normal != Vector3::z_axis() {
        return false;
    }

    let angle = ctx.me().Physics.roof_axis().angle_to(&-Vector3::z_axis());
    angle >= PI / 12.0
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::{higher_order::Repeat, taunt::TurtleSpin},
        integration_tests::helpers::{TestRunner, TestScenario},
    };
    use nalgebra::{Point3, Vector3};

    #[test]
    fn bask() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(2000.0, 0.0, 50.0),
                car_vel: Vector3::new(0.0, 2000.0, 300.0),
                ..Default::default()
            })
            .behavior(Repeat::new(TurtleSpin::new))
            .run_for_millis(4000);
        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Physics.Rotation.Roll.abs() >= 3.0);
    }
}
