use crate::{
    behavior::{
        defense::Defense,
        strike::{
            GroundedHit, GroundedHitAimContext, GroundedHitTarget, GroundedHitTargetAdjust, WallHit,
        },
    },
    eeg::{color, Drawable, Event, EEG},
    helpers::hit_angle::{blocking_angle, feasible_hit_angle_away, feasible_hit_angle_toward},
    routing::{
        behavior::FollowRoute,
        plan::{GetDollar, GroundIntercept, WallIntercept},
    },
    strategy::{Action, Behavior, Context, Context2, Priority, Scenario},
    utils::{Wall, WallRayCalculator},
};
use arrayvec::ArrayVec;
use common::{prelude::*, PrettyPrint, Time};
use nalgebra::{Point2, Point3, Vector2};
use nameof::name_of_type;
use ordered_float::NotNan;
use std::f32::consts::PI;

pub struct TepidHit;

impl TepidHit {
    pub fn new() -> Self {
        Self
    }
}

impl Behavior for TepidHit {
    fn name(&self) -> &str {
        name_of_type!(TepidHit)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let (ctx, eeg) = ctx.split();

        let mut hits = ArrayVec::<[_; 4]>::new();
        hits.push(ground(&ctx, eeg));
        hits.push(wall(&ctx, eeg));

        let hit = hits
            .into_iter()
            .flatten()
            .min_by_key(|&(duration, ref typ)| {
                eeg.log(
                    self.name(),
                    format!("{:?} would take {}", typ, Time(duration).pretty()),
                );
                NotNan::new(duration).unwrap()
            });

        match hit {
            Some((_, HitType::Wall)) => Action::tail_call(chain!(Priority::Strike, [
                FollowRoute::new(
                    WallIntercept::new()
                        .must_be_wall(true)
                        .must_be_side_wall(true)
                ),
                WallHit::new(),
            ])),
            Some((_, HitType::Ground)) => Action::tail_call(chain!(Priority::Strike, [
                FollowRoute::new(GroundIntercept::new()),
                GroundedHit::hit_towards(time_wasting_hit),
            ])),
            None => Action::tail_call(FollowRoute::new(GetDollar::smart(&ctx, eeg))),
        }
    }
}

fn ground(ctx: &Context2<'_, '_>, eeg: &mut EEG) -> Option<(f32, HitType)> {
    let intercept =
        GroundIntercept::calc_intercept(&ctx.me().into(), ctx.scenario.ball_prediction())?;

    if dangerous_back_wall_with_little_boost(ctx, intercept.loc) {
        eeg.log(name_of_type!(TepidHit), "too dangerous with no boost");
        return None;
    }
    Some((intercept.t, HitType::Ground))
}

fn dangerous_back_wall_with_little_boost(
    ctx: &Context2<'_, '_>,
    intercept_loc: Point3<f32>,
) -> bool {
    let own_goal = ctx.game.own_goal();
    let enemy_goal = ctx.game.enemy_goal();
    let near_back_wall = enemy_goal.is_y_within_range(intercept_loc.y, ..2500.0);
    let approach_angle = own_goal
        .normal_2d
        .angle_to(&(intercept_loc.to_2d() - ctx.me().Physics.loc_2d()));
    near_back_wall && approach_angle.abs() < PI / 3.0 && ctx.me().Boost < 34
}

fn wall<'ball>(ctx: &Context2<'_, 'ball>, eeg: &mut EEG) -> Option<(f32, HitType)> {
    let intercept = match WallIntercept::new()
        .must_be_wall(true)
        .must_be_side_wall(true)
        .calc_intercept(ctx)
    {
        Ok(i) => i,
        Err(reason) => {
            eeg.log(
                name_of_type!(TepidHit),
                format!("wall: no because {}", reason),
            );
            return None;
        }
    };
    eeg.log(
        name_of_type!(TepidHit),
        format!("wall: intercept is {}", intercept.loc.pretty()),
    );

    Some((intercept.t, HitType::Wall))
}

#[derive(Ord, PartialOrd, Eq, PartialEq, Debug)]
enum HitType {
    Ground,
    Wall,
}

fn time_wasting_hit(ctx: &mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> {
    let me_loc = ctx.car.Physics.loc_2d();
    let ball_loc = ctx.intercept_ball_loc.to_2d();
    let offense_aim = ctx.game.enemy_back_wall_center();
    let defense_avoid = ctx.game.own_back_wall_center();

    let naive_offense = (ball_loc - me_loc).angle_to(&(offense_aim - me_loc));
    let naive_defense = (ball_loc - me_loc).angle_to(&(defense_avoid - me_loc));

    let (aim_loc, target_adjust);
    if (ball_loc.y - defense_avoid.y).abs() < 500.0
        || ((ball_loc.y - defense_avoid.y).abs() < 1500.0
            && ctx.scenario.possession() < Scenario::POSSESSION_CONTESTABLE)
    {
        ctx.eeg.track(Event::TepidHitBlockAngleToGoal);
        ctx.eeg
            .draw(Drawable::print("blocking angle to goal", color::GREEN));
        let target_angle = blocking_angle(
            ctx.intercept_ball_loc.to_2d(),
            me_loc,
            defense_avoid,
            PI / 12.0,
        );
        aim_loc = ball_loc - Vector2::unit(target_angle) * 4000.0;
        target_adjust = if Defense::is_between_ball_and_own_goal(ctx.game, ctx.car, ctx.scenario) {
            ctx.eeg.draw(Drawable::print("straight-on", color::GREEN));
            GroundedHitTargetAdjust::StraightOn
        } else {
            ctx.eeg.draw(Drawable::print("rough aim", color::GREEN));
            GroundedHitTargetAdjust::RoughAim
        };
    } else if naive_offense.abs() < naive_defense.abs() {
        ctx.eeg.track(Event::TepidHitTowardEnemyGoal);
        ctx.eeg
            .draw(Drawable::print("toward enemy goal", color::GREEN));
        aim_loc = offensive_aim(ctx);
        target_adjust = GroundedHitTargetAdjust::RoughAim;
    } else {
        ctx.eeg.track(Event::TepidHitAwayFromOwnGoal);
        ctx.eeg
            .draw(Drawable::print("away from own goal", color::GREEN));
        aim_loc = feasible_hit_angle_away(ball_loc, me_loc, defense_avoid, PI / 6.0);
        target_adjust = GroundedHitTargetAdjust::RoughAim;
    };

    let aim_loc = WallRayCalculator::calculate(ball_loc, aim_loc);
    let aim_wall = WallRayCalculator::wall_for_point(ctx.game, aim_loc);
    if aim_wall == Wall::OwnGoal {
        ctx.eeg.log(name_of_type!(TepidHit), "refusing to own goal");
        return Err(());
    }

    Ok(
        GroundedHitTarget::new(ctx.intercept_time, target_adjust, aim_loc)
            .jump(!is_chippable(ctx, aim_loc))
            .dodge(TepidHit::should_dodge(ctx, aim_wall)),
    )
}

fn offensive_aim(ctx: &mut GroundedHitAimContext<'_, '_>) -> Point2<f32> {
    let me_loc = ctx.car.Physics.loc_2d();
    let ball_loc = ctx.intercept_ball_loc.to_2d();
    let ideal_aim = ctx.game.enemy_back_wall_center();

    // These are our choices. Take the one the enemy isn't defending.
    let progress = feasible_hit_angle_toward(ball_loc, me_loc, ideal_aim, PI / 6.0);
    let easy = ball_loc + (ball_loc - me_loc);

    let enemy = some_or_else!(ctx.scenario.primary_enemy(), {
        return progress;
    });
    let enemy_loc = enemy.Physics.loc_2d();
    let enemy_forward_axis = enemy.Physics.forward_axis_2d();

    let enemy_look_angle = enemy_forward_axis.angle_to(&(ball_loc - enemy_loc));
    ctx.eeg.print_angle("enemy_look_angle", enemy_look_angle);

    let enemy_defending = enemy.OnGround && enemy_look_angle.abs() < PI / 3.0;
    if !enemy_defending {
        return progress;
    }

    let rot_to_enemy = (ball_loc - me_loc).rotation_to(&(enemy_loc - ball_loc));
    let rot_progress = (ball_loc - me_loc).rotation_to(&(progress - ball_loc));
    let rot_easy = (ball_loc - me_loc).rotation_to(&(easy - ball_loc));

    ctx.eeg.print_angle("rot_to_enemy", rot_to_enemy.angle());
    ctx.eeg.print_angle("rot_progress", rot_progress.angle());
    ctx.eeg.print_angle("rot_easy", rot_easy.angle());

    // If the enemy isn't near the better aim, hit it there
    if rot_progress.angle_to(&rot_to_enemy).abs() >= PI / 6.0 {
        return progress;
    }

    // If the enemy is there, choose the spot where the enemy is not.
    if rot_progress.angle_to(&rot_to_enemy).abs() < rot_easy.angle_to(&rot_to_enemy).abs() {
        easy
    } else {
        progress
    }
}

fn is_chippable(ctx: &mut GroundedHitAimContext<'_, '_>, aim_loc: Point2<f32>) -> bool {
    let shot_angle = ctx
        .car
        .Physics
        .forward_axis_2d()
        .angle_to(&(aim_loc - ctx.car.Physics.loc_2d()).to_axis())
        .abs();

    let goalward_angle = (ctx.intercept_ball_loc.to_2d() - ctx.car.Physics.loc_2d())
        .angle_to(&(ctx.game.enemy_goal().center_2d - ctx.intercept_ball_loc.to_2d()))
        .abs();

    // Target a pretty specific scenario in the enemy corner, where you roll the
    // ball around the side wall without jumping so you can quickly recover and dish
    // it in.
    ctx.intercept_ball_loc.x.abs() >= 3000.0
        && (ctx.game.enemy_goal().center_2d.y - ctx.intercept_ball_loc.y).abs() < 2000.0
        && ctx.intercept_ball_loc.z < 130.0
        && shot_angle < PI / 4.0
        && goalward_angle < PI / 2.0
}

impl TepidHit {
    pub fn should_dodge(ctx: &mut GroundedHitAimContext<'_, '_>, aim_wall: Wall) -> bool {
        // Don't dodge when hitting it into the back wall since that would probably put
        // us even further out of position.
        if aim_wall != Wall::EnemyBackWall {
            return true;
        }
        let enemy_goal = ctx.game.enemy_goal();
        if !enemy_goal.is_y_within_range(ctx.scenario.ball_prediction().start().loc.y, ..2000.0) {
            return true;
        }
        false
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{eeg::Event, integration_tests::TestRunner};
    use brain_test_data::recordings;
    use common::{prelude::*, rl};
    use nalgebra::Point2;

    #[test]
    fn tepid_save() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::TEPID_SAVE, 187.5)
            .soccar()
            .run_for_millis(2500);

        assert!(!test.enemy_has_scored());
        let packet = test.sniff_packet();
        let own_goal = Point2::new(0.0, -rl::FIELD_MAX_Y);
        let goal_to_ball_dist = (packet.GameBall.Physics.loc_2d() - own_goal).norm();
        assert!(goal_to_ball_dist >= 1000.0);
        test.examine_events(|events| {
            assert!(events.contains(&Event::TepidHitBlockAngleToGoal));
        });
    }

    #[test]
    fn dont_pass_to_opponent() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::DONT_PASS_TO_OPPONENT, 459.0)
            .starting_boost(0.0)
            .soccar()
            .run_for_millis(4000);

        let packet = test.sniff_packet();
        let ball_loc = packet.GameBall.Physics.loc();
        assert!(ball_loc.y >= 0.0);
    }
}
