use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::{simple_steer_towards, GroundAccelToLoc, QuickJumpAndDodge};
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use utils::{enemy_goal_center, one_v_one, ExtendPhysics};

pub struct Shoot;

impl Shoot {
    pub fn new() -> Shoot {
        Shoot
    }
}

impl Behavior for Shoot {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        let target_loc =
            intercept.ball_loc + (intercept.ball_loc - enemy_goal_center()).normalize() * 100.0;
        let target_dist = (target_loc - me.Physics.loc()).norm();

        eeg.draw(Drawable::print("Shoot", color::YELLOW));
        eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        eeg.draw(Drawable::GhostCar(target_loc, me.Physics.rot()));

        if target_dist <= 300.0 {
            let angle = simple_steer_towards(&me.Physics, enemy_goal_center());
            return Action::call(QuickJumpAndDodge::begin(packet).yaw(angle));
        }

        if !me.OnGround {
            eeg.draw(Drawable::print("I'm scared", color::RED));
            let steer = simple_steer_towards(&me.Physics, enemy_goal_center());
            return Action::Yield(rlbot::PlayerInput {
                Throttle: 1.0,
                Steer: steer,
                ..Default::default()
            });
        }

        // TODO: this is not how this works…
        let mut child =
            GroundAccelToLoc::new(target_loc, packet.GameInfo.TimeSeconds + intercept.time);
        child.execute(packet, eeg)
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::shoot::Shoot;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    fn awkwardly_angled_breakaway() {
        let test = TestRunner::start(
            Shoot::new(),
            TestScenario::from_collect_row("570.0018	61.44401	-2595.4697	94.76237	0.4878059	0.18810439	-2.468271	-345.76355	848.0587	-6.6958303	-5.441	-2.2591	1.1357	1352.9867	-4963.935	18.86079	-0.02751578	0.14409833	0.07267234	590.5667	116.87245	5.3080044	0.17997983	0.10499983	2.2173882	1081.7532	-5051.4766	37.36723	0.628261	-1.6557405	-0.10153035	-12.958379	-17.438751	21.12848	-0.21157923	-0.12765418	-0.0040784255"),
//            TestScenario::from_collect_row("	-0.02751578	0.14409833	0.07267234	590.5667	116.87245	5.3080044	0.17997983	0.10499983	2.2173882	1081.7532	-5051.4766	37.36723	0.628261	-1.6557405	-0.10153035	-12.958379	-17.438751	21.12848	-0.21157923	-0.12765418	-0.0040784255"),
//            TestScenario {
//                ball_loc: Vector3::new(60.0, -2500.0, 90.0),
//                ball_vel: Vector3::new(-350.0, 900.0, 0.0),
//                car_loc: Vector3::new(1500.0, -5000.0, 0.0),
//                car_vel: Vector3::new(0.0, 0.0, 0.0),
//                ..Default::default()
//            }
        );

        test.sleep_millis(7000);

        assert!(test.has_scored());
    }

    #[test]
    fn fast_falling_ball() {
        let test = TestRunner::start(
            Shoot::new(),
            TestScenario {
                ball_loc: Vector3::new(3862.044, 1163.3925, 1456.4243),
                ball_vel: Vector3::new(2532.4116, 897.6915, 396.8566),
                car_loc: Vector3::new(1530.9783, 45.435856, 16.924282),
                car_rot: Rotation3::from_unreal_angles(-0.010162623, 0.28551218, -0.0006711166),
                car_vel: Vector3::new(1301.4751, 366.96378, 9.762962),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);
        assert!(test.has_scored());
    }
}
