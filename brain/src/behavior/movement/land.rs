use crate::{
    eeg::{color, Drawable},
    routing::{models::CarState, plan::avoid_goal_wall_waypoint},
    strategy::{Action, Behavior, Context},
    utils::geometry::Plane,
};
use common::{kinematics::kinematic, prelude::*, rl};
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

        // If we're way out of position, boost back towards net.
        let panic_boost = Self::panic_retreat_boost(ctx);

        if me.Physics.ang_vel().norm() >= 5.4 {
            // This is a minor hack for statelessness. A car's max angular velocity is 5.5
            // rad/sec, so if we're near that limit, we're probably in the middle of a
            // dodge. Just let it happen.
            ctx.eeg
                .draw(Drawable::print("waiting until after flip", color::GREEN));
            return Action::Yield(common::halfway_house::PlayerInput {
                // Handbrake, in case we'll be landing on a wall and want to recover without losing
                // speed. Don't handbrake near the ground, since it will mess up steering. Don't
                // handbrake in a goal, since it's not useful to conserve momentum there.
                Handbrake: me.Physics.loc().z >= 50.0
                    && ctx.game.is_inside_field(me.Physics.loc_2d()),
                Boost: panic_boost,
                ..Default::default()
            });
        }

        ctx.eeg.draw(Drawable::print("air rolling", color::GREEN));

        let (plane, landing_time) = find_landing_plane(ctx);
        ctx.eeg.print_value("plane", plane.normal);
        let want_to_boost_down = me.Boost > 0 && landing_time >= 0.6 && !panic_boost;

        // Point the nose of the car along the surface we're landing on.
        let forward = {
            let facing_2d = choose_facing_2d(ctx);
            let salvable_vel = plane.project_vector(&me.Physics.vel());
            let facing = if salvable_vel.z < 0.0 && salvable_vel.norm() >= 800.0 {
                // If there's momentum to conserve, do so.
                salvable_vel
            } else {
                // Add additional bias towards driving down the wall.
                facing_2d.to_3d().into_inner() - Vector3::z()
            };
            plane.project_vector(&facing).to_axis()
        };

        let (target_forward, boost_down);
        if want_to_boost_down {
            target_forward = forward.rotation_to(&-Vector3::z_axis()).powf(0.8) * forward;

            let nose_down_angle = me.Physics.forward_axis().angle_to(&-Vector3::z_axis());
            boost_down = nose_down_angle < PI / 3.0;

            ctx.eeg.draw(Drawable::print("boosting down", color::GREEN));
            ctx.eeg.print_time("landing_time", landing_time);
            ctx.eeg.print_angle("nose_down_angle", nose_down_angle);
        } else {
            target_forward = forward;
            boost_down = false;

            ctx.eeg.draw(Drawable::print("just floating", color::GREEN));
        }

        let (pitch, yaw, roll) = dom::get_pitch_yaw_roll(me, target_forward, plane.normal);
        Action::Yield(common::halfway_house::PlayerInput {
            Throttle: 1.0,
            Pitch: pitch,
            Yaw: yaw,
            Roll: roll,
            Boost: (boost_down || panic_boost)
                && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED,
            Handbrake: will_be_skidding_on_landing(ctx, plane),
            ..Default::default()
        })
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
fn find_landing_plane<'ctx>(ctx: &mut Context<'ctx>) -> (&'ctx Plane, f32) {
    // This routine assumes the field is fully convex (or concave I guess, since
    // we're inside it?)

    let start_loc = ctx.me().Physics.loc();
    let start_vel = ctx.me().Physics.vel();

    // If we're outside the field, we're probably inside a goal. The goals are not
    // convex(/convace?) so the below check won't work. Just orient to the ground
    // instead since that seems to produce the best results.
    if !ctx.game.is_inside_field(start_loc.to_2d()) {
        return (ctx.game.pitch().ground(), 0.0);
    }

    for time in (0..40).into_iter().map(|x| x as f32 / 20.0) {
        let (loc, _vel) = kinematic(start_vel, Vector3::z() * rl::GRAVITY, time);
        let loc = start_loc + loc;
        let plane = ctx.game.pitch().closest_plane(&loc);
        if plane.distance_to_point(&loc) < 50.0 {
            return (plane, time);
        }
    }

    // Fallback
    (ctx.game.pitch().ground(), 2.0)
}

impl Land {
    pub fn panic_retreat_boost(ctx: &mut Context<'_>) -> bool {
        let own_goal_loc = ctx.game.own_goal().center_2d;
        let ball = ctx.scenario.ball_prediction().at_time_or_last(2.0);
        let ball_loc = ball.loc.to_2d();
        let car_loc = ctx.me().Physics.loc_2d();
        let car_forward_axis = ctx.me().Physics.forward_axis();

        if !ctx.game.enemy_goal().is_y_within_range(car_loc.y, ..3000.0) {
            return false;
        }

        let dist_goal_to_ball = (ball_loc - own_goal_loc).norm();
        let dist_goal_to_car = (car_loc - own_goal_loc).norm();
        let wildly_out_of_position = dist_goal_to_ball + 2000.0 < dist_goal_to_car;
        if !wildly_out_of_position {
            return false;
        }

        // Only boost if the tailpipe is facing the right way.
        let opportune_boost_dir = car_forward_axis
            .angle_to(&(own_goal_loc - car_loc).to_3d(0.0).to_axis())
            .abs();
        opportune_boost_dir < PI / 4.0
    }
}

fn will_be_skidding_on_landing(ctx: &mut Context<'_>, plane: &Plane) -> bool {
    let nose = plane.project_vector(&ctx.me().Physics.forward_axis());
    let momentum = plane.project_vector(&ctx.me().Physics.vel());
    nose.normalize().dot(&momentum.normalize()) < 0.5
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
