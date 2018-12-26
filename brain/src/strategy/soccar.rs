use crate::{
    behavior::{
        defense::Defense, higher_order::Chain, movement::GetToFlatGround, offense::Offense,
        strike::FiftyFifty, Kickoff,
    },
    strategy::{scenario::Scenario, strategy::Strategy, Behavior, Context, Priority},
    utils::{geometry::ExtendF32, Wall},
};
use common::prelude::*;
use derive_new::new;
use nalgebra::Point2;
use nameof::name_of_type;
use simulate::linear_interpolate;
use std::f32::consts::PI;

#[derive(new)]
pub struct Soccar;

impl Strategy for Soccar {
    fn baseline(&mut self, ctx: &mut Context) -> Box<Behavior> {
        if !GetToFlatGround::on_flat_ground(ctx.me()) {
            return Box::new(GetToFlatGround::new());
        }

        match ctx.scenario.push_wall() {
            Wall::OwnGoal | Wall::OwnBackWall => return Box::new(Defense::new()),
            _ => {}
        }

        Box::new(Offense::new())
    }

    fn interrupt(&mut self, ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>> {
        // Force kickoff behavior. We can't rely on the normal routing, because it
        // doesn't account for boost pads that you pick up on the way, so it dodges and
        // goes too slow.
        if current.priority() < Priority::Force
            && (ctx.packet.GameBall.Physics.loc_2d() - Point2::origin()).norm() < 1.0
            && ctx.packet.GameBall.Physics.vel().norm() < 1.0
        {
            return Some(Box::new(Chain::new(Priority::Force, vec![Box::new(
                Kickoff::new(),
            )])));
        }

        if current.priority() < Priority::Striking
            && enemy_can_shoot(ctx)
            && GetToFlatGround::on_flat_ground(ctx.me())
            && ctx.scenario.possession().abs() < Scenario::POSSESSION_CONTESTABLE
        {
            ctx.eeg.log(
                name_of_type!(Soccar),
                format!(
                    "enemy can shoot, possession = {:.2}, going for 50/50",
                    ctx.scenario.possession(),
                ),
            );
            return Some(Box::new(Chain::new(Priority::Striking, vec![Box::new(
                FiftyFifty::new(),
            )])));
        }

        if current.priority() < Priority::Defense
            && enemy_can_shoot(ctx)
            && GetToFlatGround::on_flat_ground(ctx.me())
            && ctx.scenario.possession() < -Scenario::POSSESSION_CONTESTABLE
        {
            ctx.eeg.log(
                name_of_type!(Soccar),
                format!(
                    "enemy can shoot, possession = {:.2}, going to defense",
                    ctx.scenario.possession(),
                ),
            );
            return Some(Box::new(Chain::new(Priority::Defense, vec![Box::new(
                Defense::new(),
            )])));
        }

        None
    }
}

fn enemy_can_shoot(ctx: &mut Context) -> bool {
    let (_enemy, intercept) = match ctx.scenario.enemy_intercept() {
        Some(i) => i,
        None => return false,
    };
    let ball_loc = intercept.ball_loc.to_2d();
    let goal = ctx.game.own_goal();
    let dist_ball_to_goal = (ball_loc - goal.center_2d).norm();
    if dist_ball_to_goal >= 7500.0 {
        return false;
    }
    ctx.cars(ctx.game.enemy_team).any(|enemy| {
        let angle_car_ball = enemy.Physics.loc_2d().coords.angle_to(ball_loc.coords);
        let angle_ball_goal = ball_loc.coords.angle_to(goal.center_2d.coords);
        let angle_diff = (angle_car_ball - angle_ball_goal).normalize_angle().abs();
        let max_angle_diff =
            linear_interpolate(&[2500.0, 7500.0], &[PI / 2.0, PI / 4.0], dist_ball_to_goal);
        angle_diff < max_angle_diff
    })
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        eeg::Event,
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::Runner,
    };
    use brain_test_data::recordings;
    use common::prelude::*;
    use nalgebra::{Point3, Rotation3, Vector3};

    #[test]
    fn dont_panic_when_no_intercept() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-1185.1904, 1242.3097, 133.98555),
                ball_vel: Vector3::new(-1743.6543, 1205.1072, -55.04102),
                car_loc: Point3::new(492.75253, 567.8963, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.00958738, -3.1036267, 0.0),
                car_vel: Vector3::new(-1369.871, 12.749782, 8.351),
                ..Default::default()
            })
            .behavior(Runner::soccar())
            .run_for_millis(100);

        test.examine_events(|events| {
            assert!(!events.contains(&Event::PanicDefense));
            assert!(events.contains(&Event::Offense));
        });
    }

    #[test]
    #[ignore(note = "This only ever worked in the first place because of a hilarious tragedy.")]
    fn get_boost_on_defense_if_we_have_time() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-3112.23, -2548.45, 93.72),
                ball_vel: Vector3::new(-15.161, 716.11096, 37.961),
                car_loc: Point3::new(-2500.7, -4935.0, 17.109999),
                car_rot: Rotation3::from_unreal_angles(0.0120197795, 0.056800842, -0.0034418863),
                car_vel: Vector3::new(492.951, -132.80101, 13.231),
                enemy_loc: Point3::new(-2241.9, -2789.75, 17.02),
                enemy_rot: Rotation3::from_unreal_angles(0.009554009, 1.960959, 0.00017689279),
                enemy_vel: Vector3::new(-472.791, 1168.0409, 8.001),
                ..Default::default()
            })
            .starting_boost(0.0)
            .behavior(Runner::soccar())
            .run();

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost < 10);

        test.sleep_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost >= 50);
    }

    #[test]
    #[ignore] // Doesn't work yet
    fn get_boost_on_offense_if_we_have_time() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(810.51, 373.18, 227.86),
                ball_vel: Vector3::new(1538.241, 861.4509, -272.631),
                car_loc: Point3::new(837.23, -759.61, 18.4),
                car_rot: Rotation3::from_unreal_angles(-0.08508434, -0.4163043, 0.009112751),
                car_vel: Vector3::new(1678.101, -205.961, -160.641),
                ..Default::default()
            })
            .starting_boost(28.0)
            .behavior(Runner::soccar())
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameCars[0].Boost >= 80);
        // Go to the closest boost, don't go off to some distant corner.
        assert!(packet.GameCars[0].Physics.loc().y.abs() < 2000.0);
    }

    #[test]
    fn clear_stationary_ball() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-500.0, -5000.0, 0.0),
                car_loc: Point3::new(500.0, -5000.0, 17.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 210_f32.to_radians(), 0.0),
                ..Default::default()
            })
            .behavior(Runner::soccar())
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.Location.X < -1000.0);
    }

    #[test]
    fn clear_defensive_ball() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::CLEAR_DEFENSIVE_BALL, 53.0)
            .behavior(Runner::soccar())
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.Location.X >= 1500.0);
    }

    #[test]
    fn dont_allow_long_shot() {
        let test = TestRunner::new()
            .one_v_one(&*recordings::DONT_ALLOW_LONG_SHOT, 282.5)
            .starting_boost(0.0)
            .behavior(Runner::soccar())
            .run_for_millis(7000);

        assert!(!test.enemy_has_scored());
    }
}
