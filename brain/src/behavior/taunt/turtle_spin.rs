use crate::{
    behavior::{higher_order::Chain, movement::Yielder},
    strategy::{Action, Behavior, Context, Priority},
};
use common::{kinematics::kinematic_time, prelude::*, rl};
use dom::get_pitch_yaw_roll;
use nalgebra::Vector3;
use nameof::name_of_type;
use std::f32::consts::PI;
use vec_box::vec_box;

pub struct TurtleSpin {
    quick_chat_probability: f32,
    has_chatted: bool,
    has_oriented: bool,
}

impl TurtleSpin {
    pub fn new() -> Self {
        Self {
            quick_chat_probability: 0.0,
            has_chatted: false,
            has_oriented: false,
        }
    }

    pub fn quick_chat_probability(mut self, quick_chat_probability: f32) -> Self {
        self.quick_chat_probability = quick_chat_probability;
        self
    }
}

impl Behavior for TurtleSpin {
    fn name(&self) -> &str {
        name_of_type!(TurtleSpin)
    }

    fn priority(&self) -> Priority {
        Priority::Taunt
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        self.quick_chat(ctx);

        if ctx.me().OnGround {
            return Action::tail_call(Chain::new(self.priority(), vec_box![
                // Do nothing briefly, to let the car's suspension stabilize before we jump.
                Yielder::new(0.1, common::halfway_house::PlayerInput {
                    Boost: ctx.me().Boost > 0,
                    ..Default::default()
                }),
                // Jump.
                Yielder::new(0.2, common::halfway_house::PlayerInput {
                    Roll: 1.0,
                    Jump: true,
                    Boost: ctx.me().Boost > 0,
                    ..Default::default()
                }),
                // Then get back to what we came here to do today.
                Self::new(),
            ]));
        }

        // If we're in the middle of a dodge, go limp and wait for it to be over with.
        if ctx.me().Physics.ang_vel().norm() >= 5.45 {
            return Action::Yield(Default::default());
        }

        let car_loc = ctx.me().Physics.loc();
        let car_vel = ctx.me().Physics.vel();
        let time_to_ground =
            kinematic_time(-car_loc.z + rl::OCTANE_NEUTRAL_Z, car_vel.z, rl::GRAVITY)
                .unwrap_or(0.0);
        let dist_to_plane = ctx
            .game
            .pitch()
            .closest_plane(&car_loc)
            .distance_to_point(&car_loc);
        let turtle = dist_to_plane < 400.0 && time_to_ground < 2.0;

        if !turtle {
            return Action::Yield(common::halfway_house::PlayerInput {
                Pitch: 1.0,
                Roll: -1.0,
                Boost: true,
                ..Default::default()
            });
        }

        if !self.has_oriented {
            match self.rotate_self(ctx) {
                Some(action) => return action,
                None => self.has_oriented = true,
            }
        }

        if !ctx.me().DoubleJumped {
            return Action::Yield(common::halfway_house::PlayerInput {
                Jump: true,
                ..Default::default()
            });
        }

        Action::Yield(common::halfway_house::PlayerInput {
            Yaw: 1.0,
            Jump: !ctx.me().DoubleJumped,
            ..Default::default()
        })
    }
}

impl TurtleSpin {
    fn quick_chat(&mut self, ctx: &mut Context<'_>) {
        if self.has_chatted {
            return;
        }
        self.has_chatted = true;

        if ctx.packet.GameInfo.TimeSeconds - *ctx.last_quick_chat < 1.0 {
            return;
        }
        *ctx.last_quick_chat = ctx.packet.GameInfo.TimeSeconds;

        ctx.quick_chat(self.quick_chat_probability, &[
            rlbot::flat::QuickChatSelection::Compliments_WhatAPlay,
            rlbot::flat::QuickChatSelection::Reactions_OMG,
            rlbot::flat::QuickChatSelection::Reactions_Wow,
            rlbot::flat::QuickChatSelection::Reactions_CloseOne,
            rlbot::flat::QuickChatSelection::Reactions_NoWay,
            rlbot::flat::QuickChatSelection::Reactions_HolyCow,
            rlbot::flat::QuickChatSelection::Reactions_Whew,
            rlbot::flat::QuickChatSelection::Reactions_Siiiick,
            rlbot::flat::QuickChatSelection::Reactions_Savage,
            rlbot::flat::QuickChatSelection::Apologies_NoProblem,
            rlbot::flat::QuickChatSelection::Apologies_Sorry,
            rlbot::flat::QuickChatSelection::Apologies_MyBad,
            rlbot::flat::QuickChatSelection::Apologies_Oops,
            rlbot::flat::QuickChatSelection::Apologies_MyFault,
            rlbot::flat::QuickChatSelection::PostGame_Gg,
            rlbot::flat::QuickChatSelection::PostGame_WellPlayed,
            rlbot::flat::QuickChatSelection::PostGame_ThatWasFun,
            rlbot::flat::QuickChatSelection::PostGame_Rematch,
            rlbot::flat::QuickChatSelection::PostGame_WhatAGame,
            rlbot::flat::QuickChatSelection::PostGame_NiceMoves,
            rlbot::flat::QuickChatSelection::PostGame_EverybodyDance,
            rlbot::flat::QuickChatSelection::Custom_Useful_Faking,
        ]);
    }

    fn rotate_self(&self, ctx: &mut Context<'_>) -> Option<Action> {
        let me_forward = ctx.me().Physics.forward_axis();

        if me_forward.angle_to(&Vector3::z_axis()).abs() < PI / 4.0 && ctx.me().Boost > 25 {
            // I believe I can fly
            return None;
        }

        if should_air_roll_upside_down(ctx) {
            let (pitch, yaw, _roll) = get_pitch_yaw_roll(ctx.me(), me_forward, -Vector3::z_axis());
            return Some(Action::Yield(common::halfway_house::PlayerInput {
                Pitch: pitch,
                Yaw: yaw,
                Roll: 1.0,
                Boost: ctx.me().Boost > 0,
                ..Default::default()
            }));
        }

        None
    }
}

fn should_air_roll_upside_down(ctx: &mut Context<'_>) -> bool {
    // If we're spinning up the wall, let it happen naturally.
    let closest_plane = ctx.game.pitch().closest_plane(&ctx.me().Physics.loc());
    if closest_plane.normal != Vector3::z_axis() {
        return false;
    }

    let angle = ctx.me().Physics.roof_axis().angle_to(&-Vector3::z_axis());
    angle >= PI / 6.0
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::{higher_order::Repeat, taunt::TurtleSpin},
        integration_tests::{TestRunner, TestScenario},
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
            .run_for_millis(2000);
        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Physics.Rotation.Roll.abs() >= 3.0);
    }
}
