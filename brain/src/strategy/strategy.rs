use behavior::{Behavior, Chain, Defense, Offense, Priority};
use maneuvers::{FiftyFifty, GetToFlatGround};
use std::f32::consts::PI;
use strategy::{scenario::Scenario, Context};
use utils::{
    my_goal_center_2d, ExtendF32, ExtendPhysics, ExtendPoint3, ExtendVector2, ExtendVector3, Wall,
};

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
    // Force kickoff behavior
    if current.priority() < Priority::Force
        && ctx.packet.GameBall.Physics.loc().norm() < 1.0
        && ctx.packet.GameBall.Physics.vel().norm() < 1.0
    {
        return Some(Box::new(Chain::new(
            Priority::Force,
            vec![Box::new(FiftyFifty::new())],
        )));
    }

    if current.priority() < Priority::Save && enemy_can_shoot(ctx) {
        if ctx.scenario.possession().abs() < Scenario::POSSESSION_CONTESTABLE {
            ctx.eeg.log(format!(
                "enemy can shoot, possession = {:.2}, going for 50/50",
                ctx.scenario.possession()
            ));
            return Some(Box::new(Chain::new(
                Priority::Save,
                vec![
                    Box::new(GetToFlatGround::new()),
                    Box::new(FiftyFifty::new()),
                ],
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
    let ball_loc = enemy_intercept.1.to_2d().coords;
    if (ball_loc - my_goal_center_2d()).norm() >= 3000.0 {
        return false;
    }
    let angle_car_ball = ctx.enemy().Physics.loc().to_2d().angle_to(ball_loc);
    let angle_ball_goal = ball_loc.angle_to(my_goal_center_2d());
    let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
    angle_diff < PI / 2.0
}

#[cfg(test)]
mod integration_tests {
    use behavior::runner::PUSHED;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};
    use strategy::{runner2::BASELINE, Runner2};

    #[test]
    fn dont_panic_when_no_intercept() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-1185.1904, 1242.3097, 133.98555),
            ball_vel: Vector3::new(-1743.6543, 1205.1072, -55.04102),
            car_loc: Vector3::new(492.75253, 567.8963, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, -3.1036267, 0.0),
            car_vel: Vector3::new(-1369.871, 12.749782, 8.351),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(100);

        test.examine_eeg(|eeg| {
            assert!(!eeg
                .log
                .iter()
                .any(|x| *x == format!("{} PanicDefense", PUSHED)));
            assert!(eeg
                .log
                .iter()
                .any(|x| *x == format!("{} Offense", BASELINE)));
        });
    }
}
