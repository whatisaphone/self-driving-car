use crate::{
    behavior::movement::{simple_steer_towards, BlitzToLocation, GetToFlatGround},
    eeg::{color, Drawable, Event},
    plan::drive::rough_time_drive_to_loc,
    rules::SameBallTrajectory,
    strategy::{Action, Behavior, Context},
    utils::geometry::ExtendF32,
};
use common::{prelude::*, Distance};
use nalgebra::Point2;
use nameof::name_of_type;
use std::f32::consts::PI;

pub struct PanicDefense {
    use_boost: bool,
    same_ball_trajectory: SameBallTrajectory,
    phase: Phase,
}

enum Phase {
    Start,
    Rush {
        aim_hint: Point2<f32>,
        child: BlitzToLocation,
    },
    Turn {
        aim_hint: Point2<f32>,
        target_yaw: f32,
        start_time: f32,
    },
    Finished,
}

impl PanicDefense {
    pub fn new() -> Self {
        Self {
            use_boost: true,
            same_ball_trajectory: SameBallTrajectory::new(),
            phase: Phase::Start,
        }
    }
}

impl Behavior for PanicDefense {
    fn name(&self) -> &str {
        name_of_type!(PanicDefense)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        ctx.eeg.track(Event::PanicDefense);

        return_some!(self.same_ball_trajectory.execute_old(ctx));

        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            ctx.eeg.log(self.name(), "not on flat ground");
            return Action::tail_call(GetToFlatGround::new());
        }

        if let Some(phase) = self.next_phase(ctx) {
            self.phase = phase;
        }

        let me = ctx.me();
        ctx.eeg.print_value("use_boost", self.use_boost);

        match self.phase {
            Phase::Start => unreachable!(),
            Phase::Rush { ref mut child, .. } => {
                ctx.eeg.draw(Drawable::print("Rush", color::GREEN));
                child.execute_old(ctx)
            }
            Phase::Turn { aim_hint, .. } => {
                ctx.eeg.draw(Drawable::print("Turn", color::GREEN));
                Action::Yield(rlbot::ffi::PlayerInput {
                    Throttle: 1.0,
                    Steer: simple_steer_towards(&me.Physics, aim_hint),
                    Handbrake: true,
                    ..Default::default()
                })
            }
            Phase::Finished => Action::Return,
        }
    }
}

impl PanicDefense {
    fn blitz_loc(ctx: &mut Context<'_>, aim_loc: Point2<f32>) -> Point2<f32> {
        let goal = ctx.game.own_goal();
        let me_loc = ctx.me().Physics.loc_2d();
        let me_forward_axis = ctx.me().Physics.forward_axis_2d();

        // If we're already very close, we don't have enough time to steer.
        let future_loc = me_loc + ctx.me().Physics.vel_2d() * 1.0;
        let y_cutoff = Self::rush_y_cutoff(ctx);
        let already_close = goal.is_y_within_range(future_loc.y, ..y_cutoff);

        let approach_angle = me_forward_axis.angle_to(&-goal.normal_2d);
        let wide_angle = approach_angle.abs() >= PI / 3.0;

        if already_close && !wide_angle {
            // We don't have time to steer, just pick the closest corner.
            let left_post = Point2::new(-800.0, goal.center_2d.y);
            let right_post = Point2::new(800.0, goal.center_2d.y);
            let angle_to_left_post = me_forward_axis.angle_to(&(left_post - me_loc));
            let angle_to_right_post = me_forward_axis.angle_to(&(right_post - me_loc));
            if angle_to_left_post.abs() < angle_to_right_post.abs() {
                left_post
            } else {
                right_post
            }
        } else {
            Point2::new(800.0 * -aim_loc.x.signum(), goal.center_2d.y)
        }
    }

    pub fn rush_y_cutoff(ctx: &mut Context<'_>) -> f32 {
        ctx.me().Physics.vel().y.abs() * 0.75
    }

    fn next_phase(&mut self, ctx: &mut Context<'_>) -> Option<Phase> {
        let me = ctx.me();

        if let Phase::Start = self.phase {
            ctx.quick_chat(0.01, &[
                rlbot::flat::QuickChatSelection::Reactions_OMG,
                rlbot::flat::QuickChatSelection::Reactions_Noooo,
                rlbot::flat::QuickChatSelection::Apologies_Cursing,
                rlbot::flat::QuickChatSelection::Custom_Toxic_WasteCPU,
                rlbot::flat::QuickChatSelection::Custom_Toxic_DeAlloc,
                rlbot::flat::QuickChatSelection::Custom_Toxic_CatchVirus,
            ]);

            let aim_hint = calc_aim_hint(ctx);
            let blitz_loc = Self::blitz_loc(ctx, aim_hint);
            return Some(Phase::Rush {
                // Powerslide towards the post opposite the one we're driving to.
                aim_hint: Point2::new(
                    blitz_loc.x.signum() * -2000.0,
                    ctx.game.own_goal().center_2d.y,
                ),
                child: BlitzToLocation::new(blitz_loc),
            });
        }

        match self.phase {
            Phase::Rush { .. } | Phase::Turn { .. } => {
                if ctx
                    .game
                    .own_goal()
                    .is_y_within_range(me.Physics.loc().y, ..120.0)
                {
                    return Some(Phase::Finished);
                }
            }
            _ => {}
        }

        if let Phase::Turn {
            start_time,
            target_yaw,
            ..
        } = self.phase
        {
            let theta = (me.Physics.rot().yaw() - target_yaw).normalize_angle();
            // We're powersliding, so account for the time we'll spend recovering.
            let theta = (theta + me.Physics.ang_vel().z * 0.25).normalize_angle();
            if theta.abs() <= 15.0_f32.to_radians() {
                ctx.eeg.log(self.name(), "done, facing the right way");
                return Some(Phase::Finished);
            }
            // Fail-safe
            let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;
            if elapsed >= 0.75 {
                ctx.eeg.log(self.name(), "done, time elapsed failsafe");
                return Some(Phase::Finished);
            }
        }

        if let Phase::Rush { aim_hint, .. } = self.phase {
            let cutoff = Self::rush_y_cutoff(ctx);
            ctx.eeg
                .print_value("cutoff_distance", Distance(me.Physics.loc().y - cutoff));
            if ctx
                .game
                .own_goal()
                .is_y_within_range(me.Physics.loc().y, ..cutoff)
            {
                let target_yaw = ctx
                    .game
                    .own_goal()
                    .center_2d
                    .negated_difference_and_angle_to(aim_hint);
                return Some(Phase::Turn {
                    aim_hint,
                    start_time: ctx.packet.GameInfo.TimeSeconds,
                    target_yaw,
                });
            }
        }

        None
    }
}

fn calc_aim_hint(ctx: &mut Context<'_>) -> Point2<f32> {
    // When we reach goal, which half of the field will the ball be on?
    let own_goal = ctx.game.own_goal().center_2d;
    let time = rough_time_drive_to_loc(ctx.me(), own_goal);
    let ball = ctx.scenario.ball_prediction().at_time_or_last(time).loc;
    Point2::new(ball.x.signum() * 2000.0, own_goal.y)
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::defense::PanicDefense,
        integration_tests::{TestRunner, TestScenario},
    };
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};
    use std::f32::consts::PI;

    #[test]
    fn panic_defense() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(500.0, -1000.0, 17.01),
                car_rot: Rotation3::from_unreal_angles(0.0, -PI / 2.0, 0.0),
                car_vel: Vector3::new(0.0, 0.0, 0.0),
                ..Default::default()
            })
            .behavior(PanicDefense::new())
            .run_for_millis(4000);

        let packet = test.sniff_packet();
        println!("{:?}", packet.GameCars[0].Physics.vel());
        assert!(packet.GameCars[0].Physics.loc().y < -4500.0);
        assert!(packet.GameCars[0].Physics.vel().norm() < 100.0);
    }
}
