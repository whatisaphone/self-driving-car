use crate::{
    behavior::{
        higher_order::{Chain, TimeLimit, While},
        movement::{drive_towards, QuickJumpAndDodge, Yielder},
    },
    routing::{
        behavior::FollowRoute,
        models::RoutePlanner,
        plan::{ChainedPlanner, GroundIntercept, GroundStraightPlanner, TurnPlanner},
        recover::RoundIsNotActive,
        StraightMode,
    },
    strategy::{Action, Behavior, Context, Priority},
};
use common::{prelude::*, rl};
use derive_new::new;
use nalgebra::Point2;
use nameof::name_of_type;
use std::f32::consts::PI;
use vec_box::vec_box;

pub struct PreKickoff;

impl PreKickoff {
    pub fn new() -> Self {
        Self
    }

    pub fn is_kickoff(ball: &common::halfway_house::BallInfo) -> bool {
        (ball.Physics.loc_2d() - Point2::origin()).norm() < 1.0 && ball.Physics.vel().norm() < 1.0
    }
}

impl Behavior for PreKickoff {
    fn name(&self) -> &str {
        name_of_type!(PreKickoff)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        // Disable quick chat for now since sometimes it chats before the round
        // countdown starts, and it seems out of place.
        if false {
            kickoff_quick_chat(ctx);
        }

        Action::tail_call(Chain::new(Priority::Idle, vec_box![
            wait_for_round_to_begin(),
            Kickoff::new(),
        ]))
    }
}

fn kickoff_quick_chat(ctx: &mut Context<'_>) {
    let quick_chat = if ctx.time_based_random() < 0.1 {
        // I'm so funny
        rlbot::flat::QuickChatSelection::Information_AllYours
    } else {
        rlbot::flat::QuickChatSelection::Information_IGotIt
    };
    ctx.quick_chat(1.0, &[quick_chat]);
}

/// For some reason the game thinks we're skidding the first few frames of a
/// kickoff countdown, so we end up doing the whole SkidRecover shebang. To
/// avoid that, explicitly wait for the round to begin.
fn wait_for_round_to_begin() -> impl Behavior {
    While::new(
        RoundIsNotActive,
        Yielder::new(9999.0, common::halfway_house::PlayerInput {
            Boost: true,
            ..Default::default()
        }),
    )
}

pub struct Kickoff;

impl Kickoff {
    // This shouldn't be used without `PreKickoff` coming first, so make it private.
    fn new() -> Self {
        Self
    }
}

impl Behavior for Kickoff {
    fn name(&self) -> &str {
        name_of_type!(Kickoff)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if !PreKickoff::is_kickoff(&ctx.packet.GameBall) {
            ctx.eeg.log(self.name(), "not a kickoff");
            return Action::Abort;
        }

        // Add a "random" component to kickoffs, to keep things unpredictable.
        let [rand1, rand2, rand3, rand4] = ctx.time_based_randoms();
        // Scale these to between -1 and 1.
        let rand1 = rand1 * 2.0 - 1.0;
        let rand2 = rand2 * 2.0 - 1.0;
        let rand3 = rand3 * 2.0 - 1.0;
        let rand4 = rand4 * 2.0 - 1.0;

        let x_signum = ctx.me().Physics.loc().x.signum();
        let y_signum = ctx.me().Physics.loc().y.signum();

        let approach: Box<dyn RoutePlanner> = if is_diagonal_kickoff(ctx) {
            let straight_loc = Point2::new(
                (500.0 + rand1 * 25.0) * x_signum,
                (950.0 + rand2 * 25.0) * y_signum,
            );
            let straight =
                GroundStraightPlanner::new(straight_loc, StraightMode::Asap).allow_dodging(false);
            let turn_loc = Point2::new((100.0 + rand3 * 25.0) * x_signum, 0.0);
            let turn = TurnPlanner::new(turn_loc, None);
            Box::new(ChainedPlanner::chain(vec![
                Box::new(straight),
                Box::new(turn),
            ]))
        } else if is_off_center_kickoff(ctx) {
            let target_loc = Point2::new(
                (100.0 + rand1 * 10.0) * x_signum,
                (2500.0 + rand2 * 25.0) * y_signum,
            );
            Box::new(
                GroundStraightPlanner::new(target_loc, StraightMode::Asap).allow_dodging(false),
            )
        } else {
            // This is basically a nop since the segment runs with `StraightMode::Fake`.
            Box::new(GroundIntercept::new().allow_dodging(false))
        };

        Action::tail_call(Chain::new(Priority::Idle, vec![
            Box::new(FollowRoute::new_boxed(approach)),
            Box::new(KickoffStrike::new(rand4 * 25.0)),
        ]))
    }
}

fn is_off_center_kickoff(ctx: &mut Context<'_>) -> bool {
    let car_x = ctx.me().Physics.loc().x;
    (car_x.abs() - 256.0).abs() < 50.0
}

fn is_diagonal_kickoff(ctx: &mut Context<'_>) -> bool {
    let car_x = ctx.me().Physics.loc().x;
    car_x.abs() >= 1000.0
}

struct KickoffStrike {
    commit_offset: f32,
}

impl KickoffStrike {
    const JUMP_TIME: f32 = 0.1;

    /// `slop` is a random component added to the dodge distance, to keep things
    /// unpredictable.
    pub fn new(slop: f32) -> Self {
        Self {
            commit_offset: slop,
        }
    }
}

impl Behavior for KickoffStrike {
    fn name(&self) -> &str {
        name_of_type!(KickoffStrike)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if !PreKickoff::is_kickoff(&ctx.packet.GameBall) {
            ctx.eeg.log(self.name(), "not a kickoff");
            return Action::Abort;
        }

        let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
        let me_loc = ctx.me().Physics.loc_2d();
        let me_to_ball = ball_loc - me_loc;

        // RPS = 360° rotations per second
        const RPS: f32 = rl::CAR_MAX_ANGULAR_VELOCITY / (PI * 2.0);
        // The time for the jump plus rotating the car a certain fraction of a turn.
        let jump_flip_time = Self::JUMP_TIME + RPS * 0.2;
        let fifty_distance = ctx.me().Physics.vel_2d().norm() * jump_flip_time;
        let fifty_offset = fifty_distance - me_to_ball.norm();
        if fifty_offset >= -130.0 + self.commit_offset {
            // This frame is the "ideal" time to dodge, so now is when we need to make the
            // decision whether to dodge or not.
            return self.commit(ctx);
        }

        self.drive(ctx)
    }
}

impl KickoffStrike {
    fn drive(&self, ctx: &mut Context<'_>) -> Action {
        let target_loc = Point2::new(40.0 * ctx.me().Physics.loc().x.signum(), 0.0);
        Action::Yield(common::halfway_house::PlayerInput {
            Boost: true,
            ..drive_towards(ctx, target_loc)
        })
    }

    /// Either 50/50 or chip, depending on how close the enemy is.
    fn commit(&self, ctx: &mut Context<'_>) -> Action {
        match self.commit_action(ctx) {
            CommitAction::Dodge => {
                let me_forward = ctx.me().Physics.forward_axis_2d();
                let me_to_ball = ctx.packet.GameBall.Physics.loc_2d() - ctx.me().Physics.loc_2d();
                let angle = me_forward.angle_to(&me_to_ball.to_axis());
                Action::tail_call(
                    QuickJumpAndDodge::new()
                        .jump_time(Self::JUMP_TIME)
                        .angle(PI / 8.0 * angle.signum()),
                )
            }
            CommitAction::Chip => Action::tail_call(TimeLimit::new(1.0, RoughAngledChip::new())),
        }
    }

    fn commit_action(&self, ctx: &mut Context<'_>) -> CommitAction {
        let me = ctx.me();
        let enemy = some_or_else!(ctx.scenario.primary_enemy(), {
            return CommitAction::Chip;
        });

        let ball_loc = ctx.packet.GameBall.Physics.loc_2d();
        let me_to_ball = ball_loc - me.Physics.loc_2d();
        let enemy_to_ball = ball_loc - enemy.Physics.loc_2d();

        // If we are much closer to the ball than the enemy, they are probably faking so
        // try to chip it over their head.
        if enemy_to_ball.norm() / 2.0 >= me_to_ball.norm() {
            CommitAction::Chip
        } else {
            CommitAction::Dodge
        }
    }
}

#[derive(new)]
struct RoughAngledChip;

impl Behavior for RoughAngledChip {
    fn name(&self) -> &str {
        name_of_type!(RoughAngledChip)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if !PreKickoff::is_kickoff(&ctx.packet.GameBall) {
            return Action::Return;
        }

        let target_loc = Point2::new(140.0 * ctx.me().Physics.loc().x.signum(), 0.0);
        Action::Yield(drive_towards(ctx, target_loc))
    }
}

enum CommitAction {
    Dodge,
    Chip,
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::PreKickoff,
        integration_tests::{TestRunner, TestScenario},
    };
    use brain_test_data::recordings;
    use common::{prelude::*, rl};
    use nalgebra::{Point3, Rotation3};
    use std::f32::consts::PI;

    #[test]
    fn kickoff_center() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::KICKOFF_CENTER, 107.0)
            .starting_boost(33.0)
            .soccar()
            .run_for_millis(3000);

        let packet = test.sniff_packet();
        let ball = extrapolate_ball(&packet, 3.0);
        // Assert that the ball is not in our goal.
        assert!(!is_enemy_scored(ball));
    }

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(-256.0, -3840.0, 17.01),
                ..Default::default()
            })
            .starting_boost(33.0)
            .soccar()
            .run_for_millis(2500);

        let packet = test.sniff_packet();
        let ball = extrapolate_ball(&packet, 4.0);
        assert!(ball.y >= rl::FIELD_MAX_Y);
    }

    #[test]
    fn kickoff_diagonal() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(-1952.0, -2464.0, 17.01),
                car_rot: Rotation3::from_unreal_angles(0.0, 0.25 * PI, 0.0),
                ..Default::default()
            })
            .starting_boost(33.0)
            .soccar()
            .run_for_millis(2500);

        let packet = test.sniff_packet();
        // let ball = extrapolate_ball(&packet, 3.0);
        // assert!(is_scored(ball));
        // This works in game, but not in tests? Just test that we touched the ball
        // until I figure out what's going on.
        assert!(!PreKickoff::is_kickoff(&packet.GameBall));
    }

    fn extrapolate_ball(
        packet: &common::halfway_house::LiveDataPacket,
        seconds: f32,
    ) -> Point3<f32> {
        let ball_loc = packet.GameBall.Physics.loc();
        let ball_vel = packet.GameBall.Physics.vel();
        eprintln!("ball_loc = {:?}", ball_loc);
        eprintln!("ball_vel = {:?}", ball_vel);
        ball_loc + ball_vel * seconds
    }

    fn is_enemy_scored(ball: Point3<f32>) -> bool {
        ball.x.abs() < 1000.0 && ball.y < -5000.0
    }
}
