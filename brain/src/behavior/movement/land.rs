use crate::{
    eeg::{color, Drawable},
    routing::{models::CarState, plan::avoid_goal_wall_waypoint},
    strategy::{Action, Behavior, Context},
    utils::geometry::Plane,
};
use common::{
    kinematics::{kinematic, kinematic_time},
    prelude::*,
    rl,
};
use derive_new::new;
use nalgebra::{Unit, Vector2, Vector3};
use nameof::name_of_type;
use std::f32::consts::PI;

#[derive(new)]
pub struct Land {
    #[new(value = "false")]
    chatted: bool,
}

impl Behavior for Land {
    fn name(&self) -> &str {
        name_of_type!(Land)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let me = ctx.me();

        if me.OnGround {
            return Action::Return;
        }

        if !self.chatted {
            ctx.quick_chat(0.01, &[
                rlbot::flat::QuickChatSelection::Reactions_Whew,
                rlbot::flat::QuickChatSelection::Reactions_Okay,
                rlbot::flat::QuickChatSelection::PostGame_Gg,
            ]);
            self.chatted = true;
        }

        if me.Physics.ang_vel().norm() >= 5.4 {
            // This is a minor hack for statelessness. A car's max angular velocity is 5.5
            // rad/sec, so if we're near that limit, we're probably in the middle of a
            // dodge. Just let it happen.
            ctx.eeg
                .draw(Drawable::print("waiting until after flip", color::GREEN));
            Action::Yield(Default::default())
        } else {
            ctx.eeg.draw(Drawable::print("air rolling", color::GREEN));

            let plane = find_landing_plane(ctx);
            ctx.eeg.print_value("plane", plane.normal);

            // Boost towards the ground if we're floating helplessly
            let time_to_ground = kinematic_time(
                -me.Physics.loc().z + rl::OCTANE_NEUTRAL_Z,
                me.Physics.vel().z,
                rl::GRAVITY,
            )
            .unwrap();
            let want_to_boost_down = me.Boost > 0 && time_to_ground >= 0.6;

            let forward = {
                let facing_2d = choose_facing_2d(ctx);
                // Bias towards driving down the wall if we're landing on a wall.
                let facing = (facing_2d.to_3d().into_inner() - Vector3::z()).to_axis();
                plane.project_vector(&facing).to_axis()
            };

            let (target_forward, boost);
            if want_to_boost_down {
                target_forward = forward.rotation_to(&-Vector3::z_axis()).powf(0.8) * forward;

                let nose_down_angle = me.Physics.forward_axis().angle_to(&-Vector3::z_axis());
                boost = nose_down_angle < PI / 3.0;

                ctx.eeg.draw(Drawable::print("boosting down", color::GREEN));
                ctx.eeg.print_time("time_to_ground", time_to_ground);
                ctx.eeg.print_angle("nose_down_angle", nose_down_angle);
            } else {
                target_forward = forward;
                boost = false;

                ctx.eeg.draw(Drawable::print("just floating", color::GREEN));
            }

            let (pitch, yaw, roll) = dom::get_pitch_yaw_roll(me, target_forward, plane.normal);
            Action::Yield(common::halfway_house::PlayerInput {
                Throttle: 1.0,
                Pitch: pitch,
                Yaw: yaw,
                Roll: roll,
                Boost: boost,
                ..Default::default()
            })
        }
    }
}

impl Land {
    /// How far deep in enemy territory are we?
    pub(super) fn defensiveness(ctx: &mut Context<'_>) -> f32 {
        let safe = ctx.game.own_back_wall_center();
        let danger = ctx.game.enemy_back_wall_center();
        let me_loc = ctx.me().Physics.loc_2d();
        (me_loc - danger).norm() / (me_loc - safe).norm()
    }
}

fn choose_facing_2d(ctx: &mut Context<'_>) -> Unit<Vector2<f32>> {
    let me = ctx.me();

    if me.Physics.loc().y.abs() >= ctx.game.field_max_y() {
        // If we're going to land in the goal, land in a convenient direction to
        // immediately drive out of the goal towards the ball.
        ctx.eeg
            .draw(Drawable::print("landing in a goal", color::GREEN));
        face_the_ball(ctx)
    } else if me.Physics.vel_2d().norm() < 800.0 {
        // If we're not moving much, we have no momentum to conserve, so face the ball.
        ctx.eeg.draw(Drawable::print("no momentum", color::GREEN));
        face_the_ball(ctx)
    } else if Land::defensiveness(ctx) >= 7.0 {
        // If we're playing defense, forget momentum, try to stay ready to challenge the
        // ball.
        ctx.eeg
            .draw(Drawable::print("defensive positioning", color::GREEN));
        face_the_ball(ctx)
    } else {
        // Conserve our momentum (i.e. don't skid on landing)
        ctx.eeg
            .draw(Drawable::print("conserving momentum", color::GREEN));
        me.Physics.vel_2d()
    }
    .to_axis()
}

fn face_the_ball(ctx: &mut Context<'_>) -> Vector2<f32> {
    let me = ctx.me();
    let mut start = CarState::from(me);
    start.loc += start.vel * 1.0;
    let ball_loc = ctx
        .scenario
        .ball_prediction()
        .at_time_or_last(0.5)
        .loc
        .to_2d();
    let target_loc = avoid_goal_wall_waypoint(&start, ball_loc).unwrap_or(ball_loc);
    target_loc - me.Physics.loc_2d()
}

/// Simulate car freefall for increasing time intervals and try to find the
/// first wall we will penetrate.
fn find_landing_plane<'ctx>(ctx: &mut Context<'ctx>) -> &'ctx Plane {
    let start_loc = ctx.me().Physics.loc();
    let start_vel = ctx.me().Physics.vel();

    for time in (0..40).into_iter().map(|x| x as f32 / 20.0) {
        let (loc, _vel) = kinematic(start_vel, Vector3::z() * rl::GRAVITY, time);
        let loc = start_loc + loc;
        let plane = ctx.game.pitch().closest_plane(&loc);
        // This check assumes the field is fully convex (or concave I guess, since we're
        // inside it?)
        if plane.distance_to_point(&loc) < rl::OCTANE_NEUTRAL_Z {
            return plane;
        }
    }

    // Fallback
    ctx.game.pitch().ground()
}

#[cfg(test)]
mod demo {
    use crate::{
        behavior::movement::Land,
        integration_tests::{TestRunner, TestScenario},
    };
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    #[ignore(note = "not a test; just a demo")]
    fn land_demo() {
        TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(2500.0, 0.0, 1000.0),
                car_vel: Vector3::new(1000.0, 0.0, 500.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 0.0, 0.0),
                ..Default::default()
            })
            .behavior(Land::new())
            .run_for_millis(3000);
    }
}
