use behavior::{Behavior, Chain, Defense, Offense, Priority};
use maneuvers::{FiftyFifty, GetToFlatGround};
use std::f32::consts::PI;
use strategy::{scenario::Scenario, Context};
use utils::{my_goal_center_2d, ExtendF32, ExtendPhysics, ExtendVector2, ExtendVector3, Wall};

pub fn baseline(ctx: &mut Context) -> Box<Behavior> {
    if !GetToFlatGround::on_flat_ground(ctx.packet) {
        return Box::new(GetToFlatGround::new());
    }

    match ctx.scenario.push_wall() {
        Wall::OwnGoal | Wall::OwnBackWall => Box::new(Defense::new()),
        _ => Box::new(Offense::new()),
    }
}

pub fn override_(ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>> {
    if current.priority() < Priority::Save && enemy_can_shoot(ctx) {
        if ctx.scenario.possession().abs() < Scenario::POSSESSION_CONTESTABLE {
            ctx.eeg.log(format!(
                "enemy can shoot, possession = {:.2}, going for 50/50",
                ctx.scenario.possession()
            ));
            return Some(Box::new(Chain::new(
                Priority::Save,
                vec![Box::new(FiftyFifty::new())],
            )));
        }

        if ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE {
            ctx.eeg.log(format!(
                "enemy can shoot, possession = {:.2}, going to defense",
                ctx.scenario.possession()
            ));
            return Some(Box::new(Chain::new(
                Priority::Save,
                vec![Box::new(Defense::new())],
            )));
        }
    }

    None
}

fn enemy_can_shoot(ctx: &mut Context) -> bool {
    let enemy_intercept = match ctx.scenario.enemy_intercept() {
        Some(i) => i,
        None => return false,
    };
    let ball_loc = enemy_intercept.1.to_2d();
    if (ball_loc - my_goal_center_2d()).norm() >= 3000.0 {
        return false;
    }
    let angle_car_ball = ctx.enemy().Physics.loc().to_2d().angle_to(ball_loc);
    let angle_ball_goal = ball_loc.angle_to(my_goal_center_2d());
    let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
    angle_diff < PI / 3.0
}

#[cfg(test)]
mod tests {
    use behavior::runner::PUSHED;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::{runner2::BASELINE, Runner2};

    #[test]
    fn dont_panic_when_no_intercept() {
        let test = TestRunner::start0(TestScenario {
            enemy_loc: Vector3::new(6000.0, 6000.0, 0.0),
            ..TestScenario::from_collected_row("../logs/play.csv", 717.0)
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(100);

        test.examine_eeg(|eeg| {
            assert!(
                !eeg.log
                    .iter()
                    .any(|x| *x == format!("{} PanicDefense", PUSHED))
            );
            assert!(
                eeg.log
                    .iter()
                    .any(|x| *x == format!("{} Offense", BASELINE))
            );
        });
    }
}
