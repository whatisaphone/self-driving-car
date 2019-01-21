use crate::{
    behavior::strike::{
        GroundedHit, GroundedHitAimContext, GroundedHitTarget, GroundedHitTargetAdjust, WallHit,
    },
    eeg::{color, Drawable, Event, EEG},
    plan::hit_angle::{feasible_hit_angle_away, feasible_hit_angle_toward},
    routing::{
        behavior::FollowRoute,
        plan::{GetDollar, GroundIntercept, WallIntercept},
    },
    strategy::{Action, Behavior, Context, Context2, Priority},
    utils::{Wall, WallRayCalculator},
};
use arrayvec::ArrayVec;
use common::{prelude::*, PrettyPrint, Time};
use nalgebra::Point2;
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
            None => {
                let future_ball = ctx.scenario.ball_prediction().at_time_or_last(3.0);
                Action::tail_call(FollowRoute::new(GetDollar::new(future_ball.loc.to_2d())))
            }
        }
    }
}

fn ground(ctx: &Context2<'_, '_>, eeg: &mut EEG) -> Option<(f32, HitType)> {
    let intercept =
        GroundIntercept::calc_intercept(&ctx.me().into(), ctx.scenario.ball_prediction())?;

    let own_goal = ctx.game.own_goal();
    let enemy_goal = ctx.game.enemy_goal();
    let near_back_wall = enemy_goal.is_y_within_range(intercept.loc.y, ..2000.0);
    let approach_angle = own_goal
        .normal_2d
        .angle_to(&(intercept.loc.to_2d() - ctx.me().Physics.loc_2d()));
    if near_back_wall && approach_angle.abs() < PI / 3.0 && ctx.me().Boost < 50 {
        eeg.log(
            name_of_type!(TepidHit),
            format!("too dangerous with no boost"),
        );
        return None;
    }
    Some((intercept.t, HitType::Ground))
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

    let aim_loc = if naive_offense.abs() < naive_defense.abs() {
        ctx.eeg.track(Event::TepidHitTowardEnemyGoal);
        ctx.eeg
            .draw(Drawable::print("toward enemy goal", color::GREEN));
        feasible_hit_angle_toward(ball_loc, me_loc, offense_aim, PI / 6.0)
    } else if (ball_loc - defense_avoid).norm() < 1000.0 {
        ctx.eeg.track(Event::TepidHitBlockAngleToGoal);
        ctx.eeg
            .draw(Drawable::print("blocking angle to goal", color::GREEN));
        GroundedHit::opposite_of_self(ctx.car, ctx.intercept_ball_loc)
    } else {
        ctx.eeg.track(Event::TepidHitAwayFromOwnGoal);
        ctx.eeg
            .draw(Drawable::print("away from own goal", color::GREEN));
        feasible_hit_angle_away(ball_loc, me_loc, defense_avoid, PI / 6.0)
    };

    let aim_loc = WallRayCalculator::calculate(ball_loc, aim_loc);
    let aim_wall = WallRayCalculator::wall_for_point(ctx.game, aim_loc);
    if aim_wall == Wall::OwnGoal {
        ctx.eeg.log(name_of_type!(TepidHit), "refusing to own goal");
        return Err(());
    }

    Ok(GroundedHitTarget::new(
        ctx.intercept_time,
        GroundedHitTargetAdjust::RoughAim,
        aim_loc,
    )
    .jump(!is_chippable(ctx, aim_loc))
    .dodge(should_dodge(ctx, aim_wall)))
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

fn should_dodge(ctx: &mut GroundedHitAimContext<'_, '_>, aim_wall: Wall) -> bool {
    // Don't dodge when hitting it into the back wall since that would probably
    // put us even further out of position.
    if aim_wall != Wall::EnemyBackWall {
        return true;
    }
    let enemy_goal = ctx.game.enemy_goal();
    if !enemy_goal.is_y_within_range(ctx.scenario.ball_prediction().start().loc.y, ..1500.0) {
        return true;
    }
    false
}

#[cfg(test)]
mod integration_tests {
    use crate::{eeg::Event, integration_tests::helpers::TestRunner};
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
}
